#pragma once
#include "camera.hpp"
#include "triangle.hpp"
#include "sphere.hpp"
#include "torus.hpp"
#include "bvh.hpp"
#include "hit.hpp"
#include "material.hpp"
#include "../texture.hpp"
#include <algorithm>
#include <numeric>
#include <vector>
#include <string>

// ---------------------------------------------------------------------------
// PBR Scene
// ---------------------------------------------------------------------------

struct PBRScene {
    std::string name;
    std::vector<Triangle> triangles;
    std::vector<Sphere>   spheres;
    std::vector<Torus>    tori;
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

        // Spheres — analytic or raymarched per-object
        for (const auto& sph : spheres) {
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
                if (disc < 0.0f) continue;
                float sq = std::sqrt(disc);
                float t  = -b - sq;
                if (t < 1e-4f) t = -b + sq;
                if (t < 1e-4f || t >= best.t) continue;
                best.t = t; best.hit = true; best.matId = sph.matId;
                best.p = ray.o + ray.d * t;
                best.n = normalize(best.p - sph.center);
            }
        }

        // Tori (always raymarched)
        for (const auto& tor : tori) {
            float t = raymarchTorus(ray, tor);
            if (t > 0.f && t < best.t) {
                best.t = t; best.hit = true; best.matId = tor.matId;
                best.p = ray.o + ray.d * t;
                best.n = normalTorus(best.p, tor);
            }
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
        AABB aabb;
        for (int i = start; i < end; i++) {
            const Triangle& tri = triangles[triIndices[i]];
            for (int v = 0; v < 3; v++) aabb.expand(tri.v[v]);
        }

        int count = end - start;
        if (count <= 4) {
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

        int mid      = start + count / 2;
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
