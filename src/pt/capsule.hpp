#pragma once
#include "math.hpp"
#include <cmath>
#include <algorithm>

struct Capsule {
    Vec3  a{};                  // endpoint A
    Vec3  b{0.f, 1.f, 0.f};   // endpoint B
    float radius = 0.3f;
    int   matId  = 0;
};

inline float sdfCapsule(Vec3 p, Vec3 a, Vec3 b, float r) {
    Vec3  ab = b - a;
    Vec3  ap = p - a;
    float t  = std::clamp(dot(ap, ab) / dot(ab, ab), 0.f, 1.f);
    return length(ap - ab * t) - r;
}

inline float raymarchCapsule(const Ray& ray, const Capsule& cap) {
    float t = 1e-3f;
    for (int i = 0; i < 256 && t < 1e4f; ++i) {
        float d = sdfCapsule(ray.o + ray.d * t, cap.a, cap.b, cap.radius);
        if (d < 1e-4f) return t;
        t += d;
    }
    return -1.f;
}

inline Vec3 normalCapsule(Vec3 p, const Capsule& cap) {
    const float e = 1e-4f;
    auto sdf = [&](Vec3 q){ return sdfCapsule(q, cap.a, cap.b, cap.radius); };
    return normalize(Vec3{
        sdf({p.x+e,p.y,p.z}) - sdf({p.x-e,p.y,p.z}),
        sdf({p.x,p.y+e,p.z}) - sdf({p.x,p.y-e,p.z}),
        sdf({p.x,p.y,p.z+e}) - sdf({p.x,p.y,p.z-e})
    });
}
