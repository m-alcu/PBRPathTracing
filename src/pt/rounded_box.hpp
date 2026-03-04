#pragma once
#include "math.hpp"
#include <cmath>

struct RoundedBox {
    Vec3  center{};
    Vec3  half{0.5f, 0.5f, 0.5f};  // half-extents (must exceed cornerRadius per axis)
    float cornerRadius = 0.1f;
    int   matId        = 0;
};

inline float sdfRoundedBox(Vec3 p, Vec3 c, Vec3 half, float r) {
    Vec3 q = {std::fabs(p.x - c.x) - (half.x - r),
              std::fabs(p.y - c.y) - (half.y - r),
              std::fabs(p.z - c.z) - (half.z - r)};
    Vec3 pos = {std::max(q.x, 0.f), std::max(q.y, 0.f), std::max(q.z, 0.f)};
    return length(pos) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.f) - r;
}

inline float raymarchRoundedBox(const Ray& ray, const RoundedBox& rb) {
    float t = 1e-3f;
    for (int i = 0; i < 256 && t < 1e4f; ++i) {
        float d = sdfRoundedBox(ray.o + ray.d * t, rb.center, rb.half, rb.cornerRadius);
        if (d < 1e-4f) return t;
        t += d;
    }
    return -1.f;
}

inline Vec3 normalRoundedBox(Vec3 p, const RoundedBox& rb) {
    const float e = 1e-4f;
    auto sdf = [&](Vec3 q){ return sdfRoundedBox(q, rb.center, rb.half, rb.cornerRadius); };
    return normalize(Vec3{
        sdf({p.x+e,p.y,p.z}) - sdf({p.x-e,p.y,p.z}),
        sdf({p.x,p.y+e,p.z}) - sdf({p.x,p.y-e,p.z}),
        sdf({p.x,p.y,p.z+e}) - sdf({p.x,p.y,p.z-e})
    });
}
