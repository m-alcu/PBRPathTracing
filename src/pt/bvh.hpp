#pragma once
#include "math.hpp"
#include <algorithm>

struct AABB {
    Vec3 mn{ 1e30f,  1e30f,  1e30f};
    Vec3 mx{-1e30f, -1e30f, -1e30f};

    void expand(const Vec3& p) {
        mn.x = std::min(mn.x, p.x); mn.y = std::min(mn.y, p.y); mn.z = std::min(mn.z, p.z);
        mx.x = std::max(mx.x, p.x); mx.y = std::max(mx.y, p.y); mx.z = std::max(mx.z, p.z);
    }

    // Slab test: returns true if the ray hits the box within [tMin, tMax]
    bool intersect(const Ray& ray, float tMin, float tMax) const {
        for (int i = 0; i < 3; i++) {
            float inv = 1.0f / ray.d[i];
            float t0  = (mn[i] - ray.o[i]) * inv;
            float t1  = (mx[i] - ray.o[i]) * inv;
            if (inv < 0.0f) std::swap(t0, t1);
            tMin = std::max(tMin, t0);
            tMax = std::min(tMax, t1);
            if (tMax <= tMin) return false;
        }
        return true;
    }
};

struct BVHNode {
    AABB aabb;
    int  left     = -1;
    int  right    = -1;
    int  triStart = 0;
    int  triCount = 0;  // > 0 → leaf
};
