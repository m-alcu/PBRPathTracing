#pragma once
#include "math.hpp"
#include <cmath>

// Torus geometry (always raymarched; arbitrary symmetry axis)
struct Torus {
    Vec3  center{};
    Vec3  axis{0.f, 1.f, 0.f};  // symmetry axis (normalised); default Y-up
    float majorR = 1.0f;         // ring radius
    float minorR = 0.25f;        // tube radius
    int   matId  = 0;
};

// Torus SDF with arbitrary symmetry axis (axis must be normalised)
inline float sdfTorus(Vec3 p, Vec3 c, Vec3 axis, float R, float r) {
    Vec3  q        = p - c;
    float axProj   = dot(q, axis);
    Vec3  planePart = q - axis * axProj;
    float planeLen  = length(planePart) - R;
    return std::sqrt(planeLen * planeLen + axProj * axProj) - r;
}

inline float raymarchTorus(const Ray& ray, const Torus& tor) {
    float t = 1e-3f;
    for (int i = 0; i < 256 && t < 1e4f; ++i) {
        float d = sdfTorus(ray.o + ray.d * t, tor.center, tor.axis, tor.majorR, tor.minorR);
        if (d < 1e-4f) return t;
        t += d;
    }
    return -1.f;
}

inline Vec3 normalTorus(Vec3 p, const Torus& tor) {
    const float e = 1e-4f;
    auto sdf = [&](Vec3 q){ return sdfTorus(q, tor.center, tor.axis, tor.majorR, tor.minorR); };
    return normalize(Vec3{
        sdf({p.x+e,p.y,p.z}) - sdf({p.x-e,p.y,p.z}),
        sdf({p.x,p.y+e,p.z}) - sdf({p.x,p.y-e,p.z}),
        sdf({p.x,p.y,p.z+e}) - sdf({p.x,p.y,p.z-e})
    });
}
