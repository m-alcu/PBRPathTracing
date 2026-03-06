#pragma once
#include "math.hpp"
#include "material.hpp"
#include "rng.hpp"
#include "scene.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

// ---------------------------------------------------------------------------
// Sky / environment
// ---------------------------------------------------------------------------

// Simple sky gradient: white horizon → light blue zenith
inline Vec3 sky(const Vec3& d) {
    float t = 0.5f * (d.y + 1.0f);
    return (1.0f - t) * Vec3{1.0f, 1.0f, 1.0f} + t * Vec3{0.5f, 0.7f, 1.0f};
}

// ---------------------------------------------------------------------------
// Sampling
// ---------------------------------------------------------------------------

// Cosine-weighted hemisphere sample in local space (z = "up" / normal)
inline Vec3 sampleCosineHemisphere(float u1, float u2) {
    float r   = std::sqrt(u1);
    float phi = 2.0f * 3.14159265f * u2;
    float x   = r * std::cos(phi);
    float y   = r * std::sin(phi);
    float z   = std::sqrt(std::max(0.0f, 1.0f - u1));
    return {x, y, z};   // z aligned with surface normal
}

// Reflect v about normal n (both in world space, v pointing toward surface)
inline Vec3 reflect(const Vec3& v, const Vec3& n) {
    return v - 2.0f * dot(v, n) * n;
}

// Uniform random point inside the unit sphere (rejection sampling)
template<typename RNG>
inline Vec3 randomInUnitSphere(RNG& rng) {
    while (true) {
        Vec3 p{rng.nextFloat() * 2.0f - 1.0f,
               rng.nextFloat() * 2.0f - 1.0f,
               rng.nextFloat() * 2.0f - 1.0f};
        if (dot(p, p) < 1.0f) return p;
    }
}

// Build an orthonormal basis (T, B, N) from a surface normal N
// so that local vectors can be rotated to world space.
inline void makeONB(const Vec3& n, Vec3& t, Vec3& b) {
    Vec3 up = (std::fabs(n.z) < 0.999f) ? Vec3{0.0f, 0.0f, 1.0f}
                                         : Vec3{0.0f, 1.0f, 0.0f};
    t = normalize(cross(up, n));
    b = cross(n, t);
}

// ---------------------------------------------------------------------------
// GGX Microfacet BRDF helpers
// ---------------------------------------------------------------------------

// Schlick Fresnel approximation: F = F0 + (1-F0)*(1-cosTheta)^5
inline Vec3 fresnelSchlick(float cosTheta, const Vec3& F0) {
    float f = std::pow(1.0f - cosTheta, 5.0f);
    return F0 + (Vec3{1.0f - F0.x, 1.0f - F0.y, 1.0f - F0.z}) * f;
}

// GGX Smith G1 (masking/shadowing) for one direction.
// alpha = roughness^2 (Disney perceptual remapping)
inline float G1_GGX(float NdotV, float alpha) {
    float a2 = alpha * alpha;
    float denom = NdotV + std::sqrt(a2 + (1.0f - a2) * NdotV * NdotV);
    return 2.0f * NdotV / std::max(denom, 1e-6f);
}

// Uncorrelated Smith G2 = G1(NoL) * G1(NoV)
inline float G2_GGX(float NoL, float NoV, float alpha) {
    return G1_GGX(NoL, alpha) * G1_GGX(NoV, alpha);
}

// Snell's law refraction.
// v: unit incident direction (pointing toward surface).
// n: unit surface normal on the same side as the incident ray (dot(-v,n) >= 0).
// eta: n_incident / n_transmitted.
// Returns true and writes the refracted direction; returns false on TIR.
inline bool refractVec(const Vec3& v, const Vec3& n, float eta, Vec3& refracted) {
    float cosI  = -dot(v, n);                        // cos of incident angle (> 0)
    float sin2T = eta * eta * (1.0f - cosI * cosI);  // sin² of transmitted angle
    if (sin2T > 1.0f) return false;                  // total internal reflection
    float cosT = std::sqrt(1.0f - sin2T);
    refracted = eta * v + (eta * cosI - cosT) * n;
    return true;
}

// Sample microfacet normal from GGX NDF in local space (z = surface normal).
// alpha = roughness^2; returns unit vector in upper hemisphere.
inline Vec3 sampleGGX(float alpha, float u1, float u2) {
    float phi = 2.0f * 3.14159265f * u2;
    // Inversion of GGX CDF: cos²θ = (1-u1) / (1 + (α²-1)*u1)
    float a2 = alpha * alpha;
    float cosTheta2 = (1.0f - u1) / std::max(1.0f + (a2 - 1.0f) * u1, 1e-6f);
    float cosTheta  = std::sqrt(std::max(0.0f, cosTheta2));
    float sinTheta  = std::sqrt(std::max(0.0f, 1.0f - cosTheta2));
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// ---------------------------------------------------------------------------
// Path tracer
// ---------------------------------------------------------------------------

enum class BRDFMode { Lambertian = 0, GGX = 1 };

template<typename RNG>
inline Vec3 tracePath(Ray ray, RNG& rng,
                      const std::vector<Material>& mats,
                      const PBRScene& scene,
                      BRDFMode brdfMode = BRDFMode::GGX)
{
    Vec3 L{0.0f, 0.0f, 0.0f};
    Vec3 beta{1.0f, 1.0f, 1.0f};

    const int maxDepth = 24;
    for (int depth = 0; depth < maxDepth; ++depth) {
        Hit h = scene.intersect(ray);

        if (!h.hit) {
            // finds no geometry: we sample sky
            // returns a radiance value (white-to-blue gradient based on the ray's Y component), 
            // which gets multiplied by the current path throughput "beta" and accumulated into "L"
            // Then break exits the depth loop because the path is complete — there's nothing more to bounce off of.
            L += beta * sky(ray.d);
            break;
        }

        const Material& m = mats[h.matId];

        // Emission (area lights).
        // Guard: only add when emission is nonzero — avoids Inf*0=NaN when beta
        // has overflowed to Inf after an extreme GGX grazing-angle weight.
        if (m.emission.x > 0.0f || m.emission.y > 0.0f || m.emission.z > 0.0f)
            L += beta * m.emission;

        // Resolve albedo (texture overrides material colour)
        Vec3 albedo = m.albedo;
        if (m.albedoTex >= 0 && m.albedoTex < (int)scene.textures.size()) {
            float r, g, b;
            scene.textures[m.albedoTex].sample(h.tu, h.tv, r, g, b);
            albedo = Vec3{r, g, b} * (1.0f / 255.0f);
        }

        Vec3 wi;
        // originBias offsets the next ray origin to avoid self-intersection.
        // Refracted rays need the opposite sign (into the surface).
        Vec3 originBias = h.n * 1e-4f;
        bool isGlass = false; // glass paths skip Russian roulette

        if (brdfMode == BRDFMode::GGX) {
            if (m.transmission > 0.5f) {
                // --- Dielectric (glass) ---
                isGlass = true;
                // Determine whether the ray is entering or exiting the medium.
                bool entering = dot(ray.d, h.n) < 0.0f;
                Vec3 faceN = entering ? h.n : Vec3{-h.n.x, -h.n.y, -h.n.z};
                float eta  = entering ? (1.0f / m.ior) : m.ior;

                float cosI = -dot(ray.d, faceN);  // cos of incident angle (> 0)

                // Schlick Fresnel: r0 = ((n1-n2)/(n1+n2))²
                float r0 = (1.0f - m.ior) / (1.0f + m.ior);
                r0 *= r0;
                float R = r0 + (1.0f - r0) * std::pow(1.0f - cosI, 5.0f);

                Vec3 refracted;
                bool canRefract = refractVec(ray.d, faceN, eta, refracted);

                if (!canRefract || rng.nextFloat() < R) {
                    // Reflect (or total internal reflection)
                    wi = reflect(ray.d, faceN);
                    originBias = faceN * 1e-4f;
                } else {
                    // Refract: offset origin into the transmitted medium
                    wi = normalize(refracted);
                    originBias = Vec3{-faceN.x, -faceN.y, -faceN.z} * 1e-4f;
                }
                // Delta BSDF: throughput weight = 1 (beta unchanged)

            } else {
                // --- GGX opaque scatter ---
                Vec3 wo{-ray.d.x, -ray.d.y, -ray.d.z};
                float NoV = std::max(dot(h.n, wo), 1e-4f);

                float alpha = m.roughness * m.roughness; // Disney perceptual → GGX alpha

                // F0: 0.04 for dielectrics, albedo tint for metals
                Vec3 F0{0.04f + (albedo.x - 0.04f) * m.metallic,
                        0.04f + (albedo.y - 0.04f) * m.metallic,
                        0.04f + (albedo.z - 0.04f) * m.metallic};

                // Fresnel-based probability of specular vs diffuse bounce
                Vec3 Fapprox = fresnelSchlick(NoV, F0);
                float pSpec  = std::clamp((Fapprox.x + Fapprox.y + Fapprox.z) / 3.0f,
                                          0.05f, 0.95f);

                if (rng.nextFloat() < pSpec) {
                    // GGX specular bounce
                    Vec3 t, b;
                    makeONB(h.n, t, b);
                    Vec3 hLocal = sampleGGX(alpha, rng.nextFloat(), rng.nextFloat());
                    Vec3 wh = normalize(t * hLocal.x + b * hLocal.y + h.n * hLocal.z);

                    wi = reflect(ray.d, wh);
                    float NoL = dot(h.n, wi);
                    if (NoL <= 0.0f) break; // microfacet below surface → absorb

                    float VoH = std::max(dot(wo, wh), 1e-4f);
                    float NoH = std::max(dot(h.n, wh), 1e-4f);
                    NoL = std::max(NoL, 1e-4f);

                    Vec3  F = fresnelSchlick(VoH, F0);
                    float G = G2_GGX(NoL, NoV, alpha);
                    // Weight = BRDF * cos(θi) / pdf = F * G * VoH / (NoV * NoH)
                    beta *= F * (G * VoH / (NoV * NoH)) / pSpec;
                } else {
                    // Lambertian diffuse (metals have no diffuse)
                    Vec3 t, b;
                    makeONB(h.n, t, b);
                    Vec3 local = sampleCosineHemisphere(rng.nextFloat(), rng.nextFloat());
                    wi = normalize(t * local.x + b * local.y + h.n * local.z);
                    beta *= albedo * (1.0f - m.metallic) / (1.0f - pSpec);
                }
            }
        } else {
            // --- Lambertian + fuzz specular (original model) ---
            if (rng.nextFloat() < m.metallic) {
                // Specular: reflect + roughness fuzz
                Vec3 reflected = reflect(ray.d, h.n);
                wi = normalize(reflected + m.roughness * randomInUnitSphere(rng));
                if (dot(wi, h.n) <= 0.0f) break; // fuzz pushed below surface
            } else {
                // Lambertian diffuse
                Vec3 t, b;
                makeONB(h.n, t, b);
                Vec3 local = sampleCosineHemisphere(rng.nextFloat(), rng.nextFloat());
                wi = normalize(t * local.x + b * local.y + h.n * local.z);
            }
            beta *= albedo;
        }

        // Russian roulette from depth 3 onwards (skip for glass: beta stays 1)
        // killed 5% of glass paths per bounce from depth 3 (glass beta = 1 should never terminate early)
        if (!isGlass && depth >= 3) {
            float p = std::max({beta.x, beta.y, beta.z});
            p = std::clamp(p, 0.05f, 0.95f);
            if (rng.nextFloat() > p) break;
            beta *= (1.0f / p);
        }

        // Spawn next ray; originBias was set by the scatter branch above
        ray.o = h.p + originBias;
        ray.d = wi;
    }

    // Discard any path that produced NaN or negative radiance rather than letting
    // it permanently corrupt the accumulation buffer.
    if (!std::isfinite(L.x) || !std::isfinite(L.y) || !std::isfinite(L.z) ||
        L.x < 0.0f || L.y < 0.0f || L.z < 0.0f)
        return {0.0f, 0.0f, 0.0f};
    return L;
}
