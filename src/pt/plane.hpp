#pragma once
#include "math.hpp"
#include "hit.hpp"
#include <cmath>

// Infinite plane — intersected analytically (SDF converges in one step)
struct Plane {
    Vec3  normal{0.f, 1.f, 0.f};  // normalised; default Y-up (floor)
    float offset = 0.0f;           // dot(any_point_on_plane, normal)
    int   matId  = 0;
};

inline void intersectPlane(const Ray& ray, const Plane& pl, Hit& best) {
    float denom = dot(ray.d, pl.normal);
    if (std::fabs(denom) < 1e-6f) return;
    float t = (pl.offset - dot(ray.o, pl.normal)) / denom;
    if (t < 1e-4f || t >= best.t) return;
    best.t = t; best.hit = true; best.matId = pl.matId;
    best.p = ray.o + ray.d * t;
    best.n = denom < 0.f ? pl.normal : pl.normal * -1.f;
}
