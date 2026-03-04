#pragma once
#include "math.hpp"
#include <cmath>

struct Sphere {
    Vec3  center{};
    float radius   = 1.0f;
    int   matId    = 0;
    bool  raymarch = false;
};

inline float sdfSphere(Vec3 p, Vec3 c, float r) {
    return length(p - c) - r;
}

inline float raymarchSphere(const Ray& ray, const Sphere& s) {
    float t = 1e-3f;
    for (int i = 0; i < 256 && t < 1e4f; ++i) {
        float d = sdfSphere(ray.o + ray.d * t, s.center, s.radius);
        if (d < 1e-4f) return t;
        t += d;
    }
    return -1.f;
}
