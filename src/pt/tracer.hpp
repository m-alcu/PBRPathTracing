#pragma once
#include "math.hpp"
#include "material.hpp"
#include "rng.hpp"
#include "scene.hpp"
#include <cmath>
#include <algorithm>
#include <vector>

static constexpr float kPi    = 3.14159265f;
static constexpr float kInvPi = 0.31830988618f;

// ---------------------------------------------------------------------------
// Analytical anti-aliased checkerboard (Inigo Quilez's checkersGradBox)
// ---------------------------------------------------------------------------

// GLSL-style fract: always in [0, 1)
inline float glslFract(float x) { return x - std::floor(x); }

// Box-filtered 1-D checker integral over footprint w centred at p.
// Result in [-1, 1]; multiply two axes and scale to get [0, 1].
inline float checkerI(float p, float w) {
    float a = glslFract((p - 0.5f * w) * 0.5f) - 0.5f;
    float b = glslFract((p + 0.5f * w) * 0.5f) - 0.5f;
    return 2.0f * (std::fabs(a) - std::fabs(b)) / w;
}

// 2-D analytically box-filtered XOR checkerboard.
// px, pz: position in checker space.  wx, wz: L1 filter footprint.
// Returns blend factor in [0, 1] (0 = color A, 1 = color B).
inline float checkersGradBox(float px, float pz, float wx, float wz) {
    wx = std::max(wx, 1e-5f);
    wz = std::max(wz, 1e-5f);
    return 0.5f - 0.5f * checkerI(px, wx) * checkerI(pz, wz);
}

// ---------------------------------------------------------------------------
// Sky / environment
// ---------------------------------------------------------------------------

// Simple sky gradient: white horizon → light blue zenith
inline Vec3 sky(const Vec3& d) {
    float t = 0.5f * (d.y + 1.0f);
    return (1.0f - t) * Vec3{1.0f, 1.0f, 1.0f} + t * Vec3{0.5f, 0.7f, 1.0f};
}

// ---------------------------------------------------------------------------
// Ambient occlusion approximation (SDF march along surface normal)
//
// Marches 5 steps along the normal. At each step distance h, the SDF value d
// should equal h if space is open. If d < h the surface curves back — the
// shortfall (h-d) accumulates. An exponential weight sca *= 0.95 gives more
// importance to closer occluders. Triangles have no SDF and are omitted.
// ---------------------------------------------------------------------------

inline float sceneSDF(Vec3 p, const PBRScene& scene) {
    float d = 1e30f;
    for (const auto& sph : scene.spheres)
        d = std::min(d, sdfSphere(p, sph.center, sph.radius));
    for (const auto& tor : scene.tori)
        d = std::min(d, sdfTorus(p, tor.center, tor.axis, tor.majorR, tor.minorR));
    for (const auto& box : scene.boxes)
        d = std::min(d, sdfBox(p, box.center, box.half));
    for (const auto& cap : scene.capsules)
        d = std::min(d, sdfCapsule(p, cap.a, cap.b, cap.radius));
    for (const auto& cyl : scene.cylinders)
        d = std::min(d, sdfCylinder(p, cyl.center, cyl.axis, cyl.radius, cyl.halfHeight));
    for (const auto& rb : scene.roundedBoxes)
        d = std::min(d, sdfRoundedBox(p, rb.center, rb.half, rb.cornerRadius));
    for (const auto& pl : scene.planes)
        d = std::min(d, dot(p, pl.normal) - pl.offset);
    return d;
}

inline float calcAO(Vec3 pos, Vec3 nor, const PBRScene& scene) {
    float occ = 0.0f;
    float sca = 1.0f;
    for (int i = 0; i < 5; i++) {
        float h = 0.01f + 0.12f * (float)i / 4.0f;
        float d = sceneSDF(pos + nor * h, scene);
        occ += (h - d) * sca;
        sca *= 0.95f;
        if (occ > 0.35f) break;
    }
    return std::clamp(1.0f - 3.0f * occ, 0.0f, 1.0f) * (0.5f + 0.5f * nor.y);
}

// ---------------------------------------------------------------------------
// Sampling helpers
// ---------------------------------------------------------------------------

// Cosine-weighted hemisphere sample in local space (z = "up" / normal)
inline Vec3 sampleCosineHemisphere(float u1, float u2) {
    float r   = std::sqrt(u1);
    float phi = 2.0f * kPi * u2;
    float x   = r * std::cos(phi);
    float y   = r * std::sin(phi);
    float z   = std::sqrt(std::max(0.0f, 1.0f - u1));
    return {x, y, z};
}

// Reflect v about normal n (v pointing toward surface)
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

// Build orthonormal basis (T, B) from N
inline void makeONB(const Vec3& n, Vec3& t, Vec3& b) {
    Vec3 up = (std::fabs(n.z) < 0.999f) ? Vec3{0.0f, 0.0f, 1.0f}
                                         : Vec3{0.0f, 1.0f, 0.0f};
    t = normalize(cross(up, n));
    b = cross(n, t);
}

// ---------------------------------------------------------------------------
// GGX Microfacet BRDF helpers
// ---------------------------------------------------------------------------

// Schlick Fresnel: F = F0 + (1-F0)*(1-cosTheta)^5
inline Vec3 fresnelSchlick(float cosTheta, const Vec3& F0) {
    float f = std::pow(1.0f - cosTheta, 5.0f);
    return F0 + (Vec3{1.0f - F0.x, 1.0f - F0.y, 1.0f - F0.z}) * f;
}

// GGX NDF: D(h) = alpha^2 / (pi * ((NoH^2)*(alpha^2-1)+1)^2)
// denom = NoH^2*(a2-1)+1 has minimum a2 (at NoH=1), never reaches 0 for a2>0.
inline float D_GGX(float NoH, float alpha) {
    float a2    = alpha * alpha;
    float denom = NoH * NoH * (a2 - 1.0f) + 1.0f;
    return a2 / (kPi * denom * denom);
}

// GGX Smith G1 masking/shadowing for one direction
inline float G1_GGX(float NdotV, float alpha) {
    float a2    = alpha * alpha;
    float denom = NdotV + std::sqrt(a2 + (1.0f - a2) * NdotV * NdotV);
    return 2.0f * NdotV / std::max(denom, 1e-6f);
}

// Uncorrelated Smith G2 = G1(NoL) * G1(NoV)
inline float G2_GGX(float NoL, float NoV, float alpha) {
    return G1_GGX(NoL, alpha) * G1_GGX(NoV, alpha);
}

// Snell's law refraction. Returns false on TIR.
inline bool refractVec(const Vec3& v, const Vec3& n, float eta, Vec3& refracted) {
    float cosI  = -dot(v, n);
    float sin2T = eta * eta * (1.0f - cosI * cosI);
    if (sin2T > 1.0f) return false;
    float cosT = std::sqrt(1.0f - sin2T);
    refracted  = eta * v + (eta * cosI - cosT) * n;
    return true;
}

// Sample GGX microfacet normal in local space (z = surface normal).
// alpha = roughness^2
inline Vec3 sampleGGX(float alpha, float u1, float u2) {
    float phi   = 2.0f * kPi * u2;
    float a2    = alpha * alpha;
    float cosTheta2 = (1.0f - u1) / std::max(1.0f + (a2 - 1.0f) * u1, 1e-6f);
    float cosTheta  = std::sqrt(std::max(0.0f, cosTheta2));
    float sinTheta  = std::sqrt(std::max(0.0f, 1.0f - cosTheta2));
    return {sinTheta * std::cos(phi), sinTheta * std::sin(phi), cosTheta};
}

// MIS power heuristic (beta=2)
inline float misPower(float pdfA, float pdfB) {
    float a2 = pdfA * pdfA, b2 = pdfB * pdfB;
    return a2 / std::max(a2 + b2, 1e-10f);
}

// ---------------------------------------------------------------------------
// Sphere area light sampling helpers
// ---------------------------------------------------------------------------

// Solid-angle PDF of cone subtended by sphere from a point outside it.
// Returns 0 if from is inside the sphere.
inline float sphereLightConePdf(const Vec3& from, const Sphere& sph) {
    Vec3  toC   = sph.center - from;
    float dist2 = dot(toC, toC);
    float r2    = sph.radius * sph.radius;
    if (dist2 <= r2) return 0.0f;
    float cosMax = std::sqrt(std::max(0.0f, 1.0f - r2 / dist2));
    return 1.0f / (2.0f * kPi * (1.0f - cosMax));
}

// Sample a direction uniformly within the subtended cone, write pdf (sr^-1).
template<typename RNG>
inline Vec3 sampleSphereLight(const Vec3& from, const Sphere& sph,
                               RNG& rng, float& pdf) {
    Vec3  toC   = sph.center - from;
    float dist2 = dot(toC, toC);
    float r2    = sph.radius * sph.radius;
    if (dist2 <= r2) { pdf = 0.0f; return {}; }

    float cosMax   = std::sqrt(std::max(0.0f, 1.0f - r2 / dist2));
    float u1       = rng.nextFloat(), u2 = rng.nextFloat();
    float cosTheta = 1.0f - u1 * (1.0f - cosMax);
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));
    float phi      = 2.0f * kPi * u2;

    Vec3 zDir = normalize(toC);
    Vec3 t, b;
    makeONB(zDir, t, b);

    Vec3 dir = t * (sinTheta * std::cos(phi))
             + b * (sinTheta * std::sin(phi))
             + zDir * cosTheta;

    pdf = 1.0f / (2.0f * kPi * (1.0f - cosMax));
    return normalize(dir);
}

// ---------------------------------------------------------------------------
// Path tracer
// ---------------------------------------------------------------------------

enum class BRDFMode { Lambertian = 0, GGX = 1 };

// ---------------------------------------------------------------------------
// Direct shading render (no Monte Carlo — AO replaces indirect illumination)
//
// Mirrors IQ's analytical shading from SDL_Raymarch_SDF/main.cpp:
//   4 light terms (sun, sky, back-fill, rim) with AO modulating the ambient
//   terms.  One ray intersection + 2 shadow rays + 5 AO samples — O(1).
// ---------------------------------------------------------------------------
inline Vec3 renderDirect(const Ray& ray, const PBRScene& scene,
                         const std::vector<Material>& mats,
                         float pixelConeAngle = 0.002f,
                         bool useSun = true, bool useSky = true,
                         bool useBackFill = true, bool useRim = true)
{
    Hit h = scene.intersect(ray);
    if (!h.hit) return sky(ray.d);

    const Material& m = mats[h.matId];
    if (m.emission.x > 0.0f || m.emission.y > 0.0f || m.emission.z > 0.0f)
        return m.emission;

    // Resolve albedo — mirrors tracePath logic
    Vec3 albedo = m.albedo;
    if (m.albedoTex >= 0 && m.albedoTex < (int)scene.textures.size()) {
        float r, g, b;
        scene.textures[m.albedoTex].sample(h.tu, h.tv, r, g, b);
        albedo = Vec3{r, g, b} * (1.0f / 255.0f);
    }
    if (m.checker) {
        float denom = std::max(std::fabs(dot(ray.d, h.n)), 0.05f);
        float fw    = h.t * pixelConeAngle / denom * m.checkerScale;
        float blend = checkersGradBox(h.p.x * m.checkerScale,
                                      h.p.z * m.checkerScale, fw, fw);
        albedo = m.albedo * (1.0f - blend) + m.checkerAlbedo2 * blend;
    }

    Vec3 nor    = h.n;
    Vec3 rd     = ray.d;
    Vec3 ref    = reflect(rd, nor);

    float occ = calcAO(h.p, nor, scene);
    Vec3  lin{0.0f, 0.0f, 0.0f};

    // Light 1: Sun (directional, hard shadow)
    if (useSun) {
        const Vec3 lig = normalize(Vec3{-0.5f, 0.4f, -0.6f});
        Vec3  hal = normalize(lig - rd);
        float dif = std::clamp(dot(nor, lig), 0.0f, 1.0f);
        Hit   sh  = scene.intersect(Ray{h.p + nor * 1e-4f, lig});
        if (sh.hit) dif = 0.0f;
        float spe = std::pow(std::clamp(dot(nor, hal), 0.0f, 1.0f), 16.0f) * dif;
        spe *= 0.04f + 0.96f * std::pow(std::clamp(1.0f - dot(hal, lig), 0.0f, 1.0f), 5.0f);
        lin += albedo * 2.2f * dif * Vec3{1.3f, 1.0f, 0.7f};
        lin +=          5.0f * spe * Vec3{1.3f, 1.0f, 0.7f};
    }

    // Light 2: Sky ambient (AO-modulated) + sky specular off reflection
    if (useSky) {
        float dif = std::sqrt(std::clamp(0.5f + 0.5f * nor.y, 0.0f, 1.0f)) * occ;
        float spe = std::clamp(0.5f + 0.5f * ref.y, 0.0f, 1.0f);
        spe *= dif;
        spe *= 0.04f + 0.96f * std::pow(std::clamp(1.0f + dot(nor, rd), 0.0f, 1.0f), 5.0f);
        Hit rh = scene.intersect(Ray{h.p + nor * 1e-4f, ref});
        if (rh.hit) spe = 0.0f;
        lin += albedo * 0.6f * dif * Vec3{0.4f, 0.6f, 1.15f};
        lin +=          2.0f * spe * Vec3{0.4f, 0.6f, 1.3f};
    }

    // Light 3: Back fill (simulates ground-bounce GI, attenuates with height)
    if (useBackFill) {
        float dif = std::clamp(dot(nor, normalize(Vec3{0.5f, 0.0f, 0.6f})), 0.0f, 1.0f)
                  * std::clamp(1.0f - h.p.y, 0.0f, 1.0f) * occ;
        lin += albedo * 0.55f * dif * Vec3{0.25f, 0.25f, 0.25f};
    }

    // Light 4: Rim / SSS (fakes translucent rim glow)
    if (useRim) {
        float rim = std::pow(1.0f + dot(nor, rd), 2.0f) * occ;
        lin += albedo * 0.25f * rim * Vec3{1.0f, 1.0f, 1.0f};
    }

    return lin;
}

template<typename RNG>
inline Vec3 tracePath(Ray ray, RNG& rng,
                      const std::vector<Material>& mats,
                      const PBRScene& scene,
                      BRDFMode brdfMode = BRDFMode::GGX,
                      bool useNEE = true,
                      float pixelConeAngle = 0.002f)
{
    Vec3 L{0.0f, 0.0f, 0.0f};
    Vec3 beta{1.0f, 1.0f, 1.0f};

    // MIS tracking state (GGX mode only)
    Vec3  prevHitPos{};
    float prevBSDFPdf  = 0.0f;
    bool  prevSpecular = true;  // treat first ray as if came from a delta event

    const int maxDepth = 24;
    for (int depth = 0; depth < maxDepth; ++depth) {
        Hit h = scene.intersect(ray);

        if (!h.hit) {
            // Miss: sky doesn't need MIS (it's sampled by BSDF only)
            L += beta * sky(ray.d);
            break;
        }

        const Material& m = mats[h.matId];
        bool isEmissive = m.emission.x > 0.0f || m.emission.y > 0.0f || m.emission.z > 0.0f;

        // ----------------------------------------------------------------
        // Emission (area light hit)
        // ----------------------------------------------------------------
        if (isEmissive) {
            if (useNEE && brdfMode == BRDFMode::GGX && !prevSpecular
                    && depth > 0 && h.sphereIdx >= 0
                    && !scene.lightSphereIndices.empty()) {
                // MIS: BSDF sample hit a sphere light that NEE also samples.
                // Weight by how "surprising" it was to hit it via BSDF.
                float conePdf = sphereLightConePdf(prevHitPos,
                                                   scene.spheres[h.sphereIdx]);
                float pdfNEE  = conePdf / (float)scene.lightSphereIndices.size();
                float w       = (pdfNEE > 0.0f) ? misPower(prevBSDFPdf, pdfNEE) : 1.0f;
                L += beta * m.emission * w;
            } else {
                // No NEE, first bounce, came via glass/mirror, or Lambertian mode:
                // add with full weight.
                L += beta * m.emission;
            }
            break;  // lights don't scatter
        }

        // ----------------------------------------------------------------
        // Resolve albedo (texture overrides material colour)
        // ----------------------------------------------------------------
        Vec3 albedo = m.albedo;
        if (m.albedoTex >= 0 && m.albedoTex < (int)scene.textures.size()) {
            float r, g, b;
            scene.textures[m.albedoTex].sample(h.tu, h.tv, r, g, b);
            albedo = Vec3{r, g, b} * (1.0f / 255.0f);
        }

        // Analytical anti-aliased checkerboard (overrides albedo when enabled)
        if (m.checker) {
            // Estimate pixel footprint at hit: cone radius projected onto surface
            float denom  = std::max(std::fabs(dot(ray.d, h.n)), 0.05f);
            float fw     = h.t * pixelConeAngle / denom * m.checkerScale;
            float blend  = checkersGradBox(h.p.x * m.checkerScale,
                                           h.p.z * m.checkerScale, fw, fw);
            albedo = m.albedo * (1.0f - blend) + m.checkerAlbedo2 * blend;
        }

        Vec3  wi;
        Vec3  originBias = h.n * 1e-4f;
        bool  isGlass    = false;
        float bsdfPdf    = 0.0f;

        if (brdfMode == BRDFMode::GGX) {
            if (m.transmission > 0.5f) {
                // --------------------------------------------------------
                // Dielectric (glass) — delta BSDF
                // --------------------------------------------------------
                isGlass = true;

                bool entering = dot(ray.d, h.n) < 0.0f;
                Vec3 faceN    = entering ? h.n : Vec3{-h.n.x, -h.n.y, -h.n.z};
                float eta     = entering ? (1.0f / m.ior) : m.ior;
                float cosI    = -dot(ray.d, faceN);

                float r0 = (1.0f - m.ior) / (1.0f + m.ior); r0 *= r0;
                float R  = r0 + (1.0f - r0) * std::pow(1.0f - cosI, 5.0f);

                Vec3 refracted;
                bool canRefract = refractVec(ray.d, faceN, eta, refracted);

                if (!canRefract || rng.nextFloat() < R) {
                    wi         = reflect(ray.d, faceN);
                    originBias = faceN * 1e-4f;
                } else {
                    wi         = normalize(refracted);
                    originBias = Vec3{-faceN.x, -faceN.y, -faceN.z} * 1e-4f;
                }
                // Delta BSDF: beta unchanged (weight = 1)
                bsdfPdf = 1.0f;  // placeholder; prevSpecular=true suppresses MIS

            } else {
                // --------------------------------------------------------
                // GGX opaque surface
                // --------------------------------------------------------
                Vec3  wo    = Vec3{-ray.d.x, -ray.d.y, -ray.d.z};
                float NoV   = std::max(dot(h.n, wo), 1e-4f);
                float alpha = std::max(m.roughness * m.roughness, 1e-4f);

                Vec3 F0{0.04f + (albedo.x - 0.04f) * m.metallic,
                        0.04f + (albedo.y - 0.04f) * m.metallic,
                        0.04f + (albedo.z - 0.04f) * m.metallic};

                Vec3  Fapprox = fresnelSchlick(NoV, F0);
                float pSpec   = std::clamp((Fapprox.x + Fapprox.y + Fapprox.z) / 3.0f,
                                           0.05f, 0.95f);

                // ----------------------------------------------------
                // NEE: sample a random sphere light directly
                // ----------------------------------------------------
                int nLights = useNEE ? (int)scene.lightSphereIndices.size() : 0;
                if (nLights > 0) {
                    int  sel    = std::min((int)(rng.nextFloat() * nLights), nLights - 1);
                    int  sphIdx = scene.lightSphereIndices[sel];
                    const Sphere&   lSph = scene.spheres[sphIdx];
                    const Material& lMat = mats[lSph.matId];

                    float conePdf = sphereLightConePdf(h.p, lSph);
                    if (conePdf > 0.0f) {
                        float dirPdf;
                        Vec3  lightDir = sampleSphereLight(h.p, lSph, rng, dirPdf);
                        float NoL_nee  = dot(h.n, lightDir);

                        if (NoL_nee > 0.0f && dirPdf > 0.0f) {
                            // Shadow ray: visible if nothing blocks it OR we hit the light itself
                            Ray  sRay{h.p + h.n * 1e-4f, lightDir};
                            Hit  sHit = scene.intersect(sRay);
                            bool visible = !sHit.hit || sHit.sphereIdx == sphIdx;

                            if (visible) {
                                float pSelect = 1.0f / (float)nLights;
                                float pdfNEE  = dirPdf * pSelect;

                                // Evaluate BSDF * cos(θi) at the light direction
                                Vec3  wh_n  = normalize(wo + lightDir);
                                float NoH_n = std::max(dot(h.n, wh_n), 1e-4f);
                                float VoH_n = std::max(dot(wo, wh_n),  1e-4f);
                                float NoLn  = std::max(NoL_nee, 1e-4f);

                                Vec3  F_n   = fresnelSchlick(VoH_n, F0);
                                float D_n   = D_GGX(NoH_n, alpha);
                                float G_n   = G2_GGX(NoLn, NoV, alpha);

                                // spec BRDF * cos = F*G*D / (4*NoV)   (NoL cancels with G2 numerator)
                                Vec3 spec_cos = F_n * (G_n * D_n / (4.0f * NoV));
                                Vec3 diff_cos = albedo * (1.0f - m.metallic) * kInvPi * NoL_nee;
                                Vec3 f_cos    = pSpec * spec_cos + (1.0f - pSpec) * diff_cos;

                                // BSDF PDF at the light direction (for MIS denominator)
                                float pdfGGX_n   = D_n * NoH_n / (4.0f * VoH_n);
                                float pdfLamb_n  = NoLn * kInvPi;
                                float pdfBSDF_n  = pSpec * pdfGGX_n + (1.0f - pSpec) * pdfLamb_n;

                                float w_nee = misPower(pdfNEE, pdfBSDF_n);
                                L += beta * f_cos * lMat.emission / pdfNEE * w_nee;
                            }
                        }
                    }
                }

                // ----------------------------------------------------
                // BSDF scatter
                // ----------------------------------------------------
                if (rng.nextFloat() < pSpec) {
                    // GGX specular bounce
                    Vec3 t, b;
                    makeONB(h.n, t, b);
                    Vec3 hLocal = sampleGGX(alpha, rng.nextFloat(), rng.nextFloat());
                    Vec3 wh     = normalize(t * hLocal.x + b * hLocal.y + h.n * hLocal.z);

                    wi = reflect(ray.d, wh);
                    float NoL = dot(h.n, wi);
                    if (NoL <= 0.0f) break;

                    float VoH = std::max(dot(wo, wh), 1e-4f);
                    float NoH = std::max(dot(h.n, wh), 1e-4f);
                    NoL = std::max(NoL, 1e-4f);

                    Vec3  F = fresnelSchlick(VoH, F0);
                    float G = G2_GGX(NoL, NoV, alpha);
                    beta *= F * (G * VoH / (NoV * NoH)) / pSpec;

                    // Mixed BSDF PDF for MIS at next emission hit
                    float D_s      = D_GGX(NoH, alpha);
                    float pdfGGX_s = D_s * NoH / (4.0f * VoH);
                    float pdfLamb_s = NoL * kInvPi;
                    bsdfPdf = pSpec * pdfGGX_s + (1.0f - pSpec) * pdfLamb_s;

                } else {
                    // Lambertian diffuse bounce
                    Vec3 t, b;
                    makeONB(h.n, t, b);
                    Vec3 local = sampleCosineHemisphere(rng.nextFloat(), rng.nextFloat());
                    wi = normalize(t * local.x + b * local.y + h.n * local.z);
                    beta *= albedo * (1.0f - m.metallic) / (1.0f - pSpec);

                    float NoL_d = std::max(dot(h.n, wi), 1e-4f);
                    Vec3  wh_d  = normalize(wo + wi);
                    float NoH_d = std::max(dot(h.n, wh_d), 1e-4f);
                    float VoH_d = std::max(dot(wo,  wh_d), 1e-4f);
                    float D_d   = D_GGX(NoH_d, alpha);
                    float pdfGGX_d  = D_d * NoH_d / (4.0f * VoH_d);
                    float pdfLamb_d = NoL_d * kInvPi;
                    bsdfPdf = pSpec * pdfGGX_d + (1.0f - pSpec) * pdfLamb_d;
                }
            }
        } else {
            // --------------------------------------------------------
            // Lambertian + fuzz specular (original model, no NEE)
            // --------------------------------------------------------
            if (rng.nextFloat() < m.metallic) {
                Vec3 reflected = reflect(ray.d, h.n);
                wi = normalize(reflected + m.roughness * randomInUnitSphere(rng));
                if (dot(wi, h.n) <= 0.0f) break;
            } else {
                Vec3 t, b;
                makeONB(h.n, t, b);
                Vec3 local = sampleCosineHemisphere(rng.nextFloat(), rng.nextFloat());
                wi = normalize(t * local.x + b * local.y + h.n * local.z);
            }
            beta *= albedo;
            bsdfPdf = std::max(dot(h.n, wi), 1e-4f) * kInvPi;
        }

        // Russian roulette from depth 3 (skip for glass: beta stays near 1)
        if (!isGlass && depth >= 3) {
            float p = std::max({beta.x, beta.y, beta.z});
            p = std::clamp(p, 0.05f, 0.95f);
            if (rng.nextFloat() > p) break;
            beta *= (1.0f / p);
        }

        // Update MIS state for next bounce
        prevHitPos  = h.p;
        prevBSDFPdf = bsdfPdf;
        prevSpecular = isGlass || (brdfMode != BRDFMode::GGX);

        ray.o = h.p + originBias;
        ray.d = wi;
    }

    // Discard NaN / negative radiance to protect the accumulation buffer
    if (!std::isfinite(L.x) || !std::isfinite(L.y) || !std::isfinite(L.z) ||
        L.x < 0.0f || L.y < 0.0f || L.z < 0.0f)
        return {0.0f, 0.0f, 0.0f};
    return L;
}
