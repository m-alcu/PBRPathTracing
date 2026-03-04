#pragma once
#include "math.hpp"
#include <cmath>

struct Box {
    Vec3  center{};
    Vec3  half{0.5f, 0.5f, 0.5f};  // half-extents per axis (axis-aligned)
    int   matId = 0;
};

inline float sdfBox(Vec3 p, Vec3 c, Vec3 half) {
    Vec3 q = {std::fabs(p.x - c.x) - half.x,
              std::fabs(p.y - c.y) - half.y,
              std::fabs(p.z - c.z) - half.z};
    Vec3 pos = {std::max(q.x, 0.f), std::max(q.y, 0.f), std::max(q.z, 0.f)};
    return length(pos) + std::min(std::max(q.x, std::max(q.y, q.z)), 0.f);
}

inline float raymarchBox(const Ray& ray, const Box& box) {
    float t = 1e-3f;
    for (int i = 0; i < 256 && t < 1e4f; ++i) {
        float d = sdfBox(ray.o + ray.d * t, box.center, box.half);
        if (d < 1e-4f) return t;
        t += d;
    }
    return -1.f;
}

inline Vec3 normalBox(Vec3 p, const Box& box) {
    const float e = 1e-4f;
    auto sdf = [&](Vec3 q){ return sdfBox(q, box.center, box.half); };
    return normalize(Vec3{
        sdf({p.x+e,p.y,p.z}) - sdf({p.x-e,p.y,p.z}),
        sdf({p.x,p.y+e,p.z}) - sdf({p.x,p.y-e,p.z}),
        sdf({p.x,p.y,p.z+e}) - sdf({p.x,p.y,p.z-e})
    });
}

inline void intersectBox(const Ray &ray, const Box &box, Hit &best)
{
    float t = raymarchBox(ray, box);
    if (t > 0.f && t < best.t)
    {
        best.t = t;
        best.hit = true;
        best.matId = box.matId;
        best.p = ray.o + ray.d * t;
        best.n = normalBox(best.p, box);
    }
}