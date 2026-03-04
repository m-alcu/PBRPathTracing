#pragma once
#include "math.hpp"
#include "hit.hpp"
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

inline void intersectSphere(const Ray& ray, const Sphere& sph, Hit& best) {
    if (sph.raymarch) {
        float t = raymarchSphere(ray, sph);
        if (t > 0.f && t < best.t) {
            best.t = t; best.hit = true; best.matId = sph.matId;
            best.p = ray.o + ray.d * t;
            best.n = normalize(best.p - sph.center);
        }
    } else {
        Vec3  oc   = ray.o - sph.center;
        float b    = dot(oc, ray.d);
        float c    = dot(oc, oc) - sph.radius * sph.radius;
        float disc = b * b - c;
        if (disc < 0.0f) return;
        float sq = std::sqrt(disc);
        float t  = -b - sq;
        if (t < 1e-4f) t = -b + sq;
        if (t < 1e-4f || t >= best.t) return;
        best.t = t; best.hit = true; best.matId = sph.matId;
        best.p = ray.o + ray.d * t;
        best.n = normalize(best.p - sph.center);
    }
}
