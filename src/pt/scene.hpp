#pragma once
#include "math.hpp"
#include "material.hpp"
#include "../texture.hpp"
#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// Camera
// ---------------------------------------------------------------------------

struct Camera {
    Vec3  orbitTarget{0.0f, 0.0f, 0.0f};
    float orbitRadius    = 4.0f;
    float orbitAzimuth   = 0.0f;
    float orbitElevation = 0.2f;
    float fov = 45.0f;   // vertical FOV in degrees

    // Derived state – updated by applyOrbit() / setLookAt()
    Vec3 pos{0.0f, 0.8f, 4.0f};
    Vec3 forward{0.0f, 0.0f, -1.0f};

    void applyOrbit() {
        float ca = std::cos(orbitAzimuth),  sa = std::sin(orbitAzimuth);
        float ce = std::cos(orbitElevation), se = std::sin(orbitElevation);
        pos     = orbitTarget + Vec3{sa * ce, se, ca * ce} * orbitRadius;
        forward = normalize(orbitTarget - pos);
    }

    void setLookAt(Vec3 eye, Vec3 target) {
        orbitTarget = target;
        Vec3 d = eye - target;
        orbitRadius    = length(d);
        orbitAzimuth   = std::atan2(d.x, d.z);
        float safe = std::clamp(d.y / orbitRadius, -0.999f, 0.999f);
        orbitElevation = std::asin(safe);
        pos     = eye;
        forward = normalize(target - eye);
    }

    Ray generateRay(int px, int py, int w, int h, float du, float dv) const {
        Vec3 right = normalize(cross(forward, Vec3{0.0f, 1.0f, 0.0f}));
        if (length(right) < 0.001f) right = {1.0f, 0.0f, 0.0f};
        Vec3 up = cross(right, forward);

        float aspect = (float)w / (float)h;
        float scale  = std::tan(fov * 0.5f * 3.14159265f / 180.0f);
        float ndcX   = (2.0f * ((float)px + du) / (float)w - 1.0f) * aspect * scale;
        float ndcY   = (1.0f - 2.0f * ((float)py + dv) / (float)h) * scale;

        Vec3 dir = normalize(forward + right * ndcX + up * ndcY);
        return {pos, dir};
    }
};

// ---------------------------------------------------------------------------
// Triangle geometry
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Analytical sphere
// ---------------------------------------------------------------------------

struct Sphere {
    Vec3  center{};
    float radius = 1.0f;
    int   matId  = 0;
};

// ---------------------------------------------------------------------------
// Axis-Aligned Bounding Box
// ---------------------------------------------------------------------------

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

// ---------------------------------------------------------------------------
// Binary BVH over triangles
// ---------------------------------------------------------------------------

struct BVHNode {
    AABB aabb;
    int  left     = -1;
    int  right    = -1;
    int  triStart = 0;
    int  triCount = 0;  // > 0 → leaf
};

// ---------------------------------------------------------------------------
// PBR Scene
// ---------------------------------------------------------------------------

struct PBRScene {
    std::string name;
    std::vector<Triangle> triangles;
    std::vector<Sphere>   spheres;
    std::vector<Material> materials;
    std::vector<Texture>  textures;   // diffuse textures referenced by materials
    Camera camera;

    // BVH (built after geometry is loaded)
    std::vector<BVHNode> bvh;
    std::vector<int>     triIndices;  // reordered triangle indices

    // -----------------------------------------------------------------------
    // BVH construction
    // -----------------------------------------------------------------------

    void buildBVH() {
        if (triangles.empty()) return;
        triIndices.resize(triangles.size());
        std::iota(triIndices.begin(), triIndices.end(), 0);
        bvh.clear();
        bvh.reserve(triangles.size() * 2 + 4);
        bvh.emplace_back();           // root = index 0
        buildNode(0, 0, (int)triangles.size());
    }

    // -----------------------------------------------------------------------
    // Intersection
    // -----------------------------------------------------------------------

    Hit intersect(const Ray& ray) const {
        Hit best;

        // Triangle BVH traversal
        if (!bvh.empty())
            traverseBVH(0, ray, best);
        else
            for (int i = 0; i < (int)triangles.size(); i++)
                testTriangle(i, ray, best);

        // Spheres (usually few, brute-force is fine)
        for (const auto& sph : spheres) {
            Vec3  oc   = ray.o - sph.center;
            float b    = dot(oc, ray.d);
            float c    = dot(oc, oc) - sph.radius * sph.radius;
            float disc = b * b - c;
            if (disc < 0.0f) continue;
            float sq = std::sqrt(disc);
            float t  = -b - sq;
            if (t < 1e-4f) t = -b + sq;
            if (t < 1e-4f || t >= best.t) continue;
            best.t     = t;
            best.hit   = true;
            best.matId = sph.matId;
            best.p     = ray.o + ray.d * t;
            best.n     = normalize(best.p - sph.center);
        }
        return best;
    }

private:
    void testTriangle(int idx, const Ray& ray, Hit& best) const {
        const Triangle& tri = triangles[triIndices.empty() ? idx : triIndices[idx]];
        float t, u, v;
        if (intersectTriangle(ray, tri, t, u, v) && t < best.t) {
            best.t     = t;
            best.hit   = true;
            best.matId = tri.matId;
            best.p     = ray.o + ray.d * t;
            float w    = 1.0f - u - v;
            best.n     = normalize(tri.n[0] * w + tri.n[1] * u + tri.n[2] * v);
            best.tu    = tri.uv[0][0] * w + tri.uv[1][0] * u + tri.uv[2][0] * v;
            best.tv    = tri.uv[0][1] * w + tri.uv[1][1] * u + tri.uv[2][1] * v;
        }
    }

    void traverseBVH(int nodeIdx, const Ray& ray, Hit& best) const {
        const BVHNode& node = bvh[nodeIdx];
        if (!node.aabb.intersect(ray, 1e-4f, best.t)) return;

        if (node.triCount > 0) {
            for (int i = node.triStart; i < node.triStart + node.triCount; i++) {
                const Triangle& tri = triangles[triIndices[i]];
                float t, u, v;
                if (intersectTriangle(ray, tri, t, u, v) && t < best.t) {
                    best.t     = t;
                    best.hit   = true;
                    best.matId = tri.matId;
                    best.p     = ray.o + ray.d * t;
                    float w    = 1.0f - u - v;
                    best.n     = normalize(tri.n[0] * w + tri.n[1] * u + tri.n[2] * v);
                    best.tu    = tri.uv[0][0] * w + tri.uv[1][0] * u + tri.uv[2][0] * v;
                    best.tv    = tri.uv[0][1] * w + tri.uv[1][1] * u + tri.uv[2][1] * v;
                }
            }
        } else {
            traverseBVH(node.left,  ray, best);
            traverseBVH(node.right, ray, best);
        }
    }

    void buildNode(int nodeIdx, int start, int end) {
        // Build AABB for this range
        AABB aabb;
        for (int i = start; i < end; i++) {
            const Triangle& tri = triangles[triIndices[i]];
            for (int v = 0; v < 3; v++) aabb.expand(tri.v[v]);
        }

        int count = end - start;
        if (count <= 4) {
            // Leaf
            bvh[nodeIdx].aabb     = aabb;
            bvh[nodeIdx].triStart = start;
            bvh[nodeIdx].triCount = count;
            bvh[nodeIdx].left     = -1;
            bvh[nodeIdx].right    = -1;
            return;
        }

        // Choose split axis (largest AABB extent)
        Vec3 size{aabb.mx.x - aabb.mn.x, aabb.mx.y - aabb.mn.y, aabb.mx.z - aabb.mn.z};
        int axis = 0;
        if (size.y > size[axis]) axis = 1;
        if (size.z > size[axis]) axis = 2;

        // Sort by triangle centroid on chosen axis
        std::sort(triIndices.begin() + start, triIndices.begin() + end,
            [&, axis](int a, int b) {
                const Triangle& ta = triangles[a];
                const Triangle& tb = triangles[b];
                float ca = ta.v[0][axis] + ta.v[1][axis] + ta.v[2][axis];
                float cb = tb.v[0][axis] + tb.v[1][axis] + tb.v[2][axis];
                return ca < cb;
            });

        int mid    = start + count / 2;
        int leftIdx  = (int)bvh.size(); bvh.push_back(BVHNode{});
        int rightIdx = (int)bvh.size(); bvh.push_back(BVHNode{});

        // Write interior node (after push_back, data() may have changed)
        bvh[nodeIdx].aabb     = aabb;
        bvh[nodeIdx].left     = leftIdx;
        bvh[nodeIdx].right    = rightIdx;
        bvh[nodeIdx].triCount = 0;

        buildNode(leftIdx,  start, mid);
        buildNode(rightIdx, mid,   end);
    }
};
