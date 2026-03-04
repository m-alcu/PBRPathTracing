#pragma once
#include "math.hpp"
#include <cmath>

struct Cylinder {
    Vec3  center{};
    Vec3  axis{0.f, 1.f, 0.f};  // symmetry axis (normalised); default Y-up
    float radius     = 0.3f;
    float halfHeight = 0.5f;
    int   matId      = 0;
};

inline float sdfCylinder(Vec3 p, Vec3 c, Vec3 axis, float r, float h) {
    Vec3  q        = p - c;
    float axProj   = dot(q, axis);
    Vec3  planePart = q - axis * axProj;
    float dx = length(planePart) - r;
    float dy = std::fabs(axProj) - h;
    float dxc = std::max(dx, 0.f), dyc = std::max(dy, 0.f);
    return std::min(std::max(dx, dy), 0.f) + std::sqrt(dxc*dxc + dyc*dyc);
}

inline float raymarchCylinder(const Ray& ray, const Cylinder& cyl) {
    float t = 1e-3f;
    for (int i = 0; i < 256 && t < 1e4f; ++i) {
        float d = sdfCylinder(ray.o + ray.d * t, cyl.center, cyl.axis, cyl.radius, cyl.halfHeight);
        if (d < 1e-4f) return t;
        t += d;
    }
    return -1.f;
}

inline Vec3 normalCylinder(Vec3 p, const Cylinder& cyl) {
    const float e = 1e-4f;
    auto sdf = [&](Vec3 q){ return sdfCylinder(q, cyl.center, cyl.axis, cyl.radius, cyl.halfHeight); };
    return normalize(Vec3{
        sdf({p.x+e,p.y,p.z}) - sdf({p.x-e,p.y,p.z}),
        sdf({p.x,p.y+e,p.z}) - sdf({p.x,p.y-e,p.z}),
        sdf({p.x,p.y,p.z+e}) - sdf({p.x,p.y,p.z-e})
    });
}

inline void intersectCylinder(const Ray &ray, const Cylinder &cyl, Hit &best) {
    float t = raymarchCylinder(ray, cyl);
    if (t > 0.f && t < best.t){
        best.t = t;
        best.hit = true;
        best.matId = cyl.matId;
        best.p = ray.o + ray.d * t;
        best.n = normalCylinder(best.p, cyl);
    }
}
