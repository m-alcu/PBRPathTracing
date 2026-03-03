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
// Path tracer
// ---------------------------------------------------------------------------

template<typename RNG>
inline Vec3 tracePath(Ray ray, RNG& rng,
                      const std::vector<Material>& mats,
                      const PBRScene& scene)
{
    Vec3 L{0.0f, 0.0f, 0.0f};
    Vec3 beta{1.0f, 1.0f, 1.0f};

    const int maxDepth = 8;
    for (int depth = 0; depth < maxDepth; ++depth) {
        Hit h = scene.intersect(ray);

        if (!h.hit) {
            // Miss: sample sky
            L += beta * sky(ray.d);
            break;
        }

        const Material& m = mats[h.matId];

        // Emission (area lights)
        L += beta * m.emission;

        // --- Scatter: metallic specular or Lambertian diffuse ---
        Vec3 wi;
        if (rng.nextFloat() < m.metallic) {
            // Specular (metallic) bounce: reflect + roughness fuzz
            Vec3 reflected = reflect(ray.d, h.n);
            wi = normalize(reflected + m.roughness * randomInUnitSphere(rng));
            // Fuzz can push wi below the surface → the ray is absorbed
            if (dot(wi, h.n) <= 0.0f) break;
        } else {
            // Lambertian diffuse bounce (cosine-weighted hemisphere)
            Vec3 t, b;
            makeONB(h.n, t, b);
            Vec3 local = sampleCosineHemisphere(rng.nextFloat(), rng.nextFloat());
            wi = normalize(t * local.x + b * local.y + h.n * local.z);
        }

        // Throughput: use texture albedo if available, else material albedo
        Vec3 albedo = m.albedo;
        if (m.albedoTex >= 0 && m.albedoTex < (int)scene.textures.size()) {
            float r, g, b;
            scene.textures[m.albedoTex].sample(h.tu, h.tv, r, g, b);
            albedo = Vec3{r, g, b} * (1.0f / 255.0f);
        }
        beta *= albedo;

        // Russian roulette from depth 3 onwards
        if (depth >= 3) {
            float p = std::max({beta.x, beta.y, beta.z});
            p = std::clamp(p, 0.05f, 0.95f);
            if (rng.nextFloat() > p) break;
            beta *= (1.0f / p);
        }

        // Spawn next ray with a small epsilon offset to avoid self-intersection
        ray.o = h.p + h.n * 1e-4f;
        ray.d = wi;
    }

    return L;
}
