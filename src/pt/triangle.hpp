#pragma once
#include "math.hpp"
#include <cmath>

struct Triangle {
    Vec3  v[3];
    Vec3  n[3];           // per-vertex normals (interpolated on hit)
    float uv[3][2] = {};  // per-vertex texture coordinates (u, v in [0,1])
    int   matId = 0;
};

// Möller–Trumbore triangle intersection
inline bool intersectTriangle(const Ray& ray, const Triangle& tri,
                               float& tOut, float& uOut, float& vOut) {
    const float eps = 1e-8f;
    Vec3 e1 = tri.v[1] - tri.v[0];
    Vec3 e2 = tri.v[2] - tri.v[0];
    Vec3 h  = cross(ray.d, e2);
    float a = dot(e1, h);
    if (std::fabs(a) < eps) return false;
    float f = 1.0f / a;
    Vec3 s  = ray.o - tri.v[0];
    float u = f * dot(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    Vec3 q  = cross(s, e1);
    float v = f * dot(ray.d, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    float t = f * dot(e2, q);
    if (t < 1e-4f) return false;
    tOut = t; uOut = u; vOut = v;
    return true;
}
