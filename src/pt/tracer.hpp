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

        // --- Lambertian diffuse bounce ---

        // Build orthonormal basis aligned with hit normal
        Vec3 t, b;
        makeONB(h.n, t, b);

        // Cosine-weighted sample in hemisphere
        Vec3 local = sampleCosineHemisphere(rng.nextFloat(), rng.nextFloat());
        Vec3 wi    = normalize(t * local.x + b * local.y + h.n * local.z);

        // Throughput update:
        // BRDF = albedo / π, PDF = cos(θ) / π → beta *= albedo (they cancel)
        beta *= m.albedo;

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
