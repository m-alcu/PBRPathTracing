#include <cmath>
#include <cstdio>
#include <vector>

// pt headers (header-only, no scene_loader needed)
#include "pt/math.hpp"
#include "pt/hit.hpp"
#include "pt/rng.hpp"
#include "pt/material.hpp"
#include "pt/triangle.hpp"
#include "pt/sphere.hpp"
#include "pt/plane.hpp"
#include "pt/bvh.hpp"
#include "pt/camera.hpp"
#include "pt/tracer.hpp"   // sky, sampleCosineHemisphere, makeONB, GGX helpers, etc.

constexpr float EPS = 1e-4f;

static int failures = 0;

static bool near(float a, float b, float eps = EPS) { return std::fabs(a - b) < eps; }
static bool near(Vec3 a, Vec3 b, float eps = EPS) {
    return near(a.x, b.x, eps) && near(a.y, b.y, eps) && near(a.z, b.z, eps);
}

#define CHECK(expr) do { \
    if (!(expr)) { std::fprintf(stderr, "FAIL [%s:%d]: %s\n", __FILE__, __LINE__, #expr); ++failures; } \
    else { std::printf("PASS: %s\n", #expr); } \
} while(0)

// ============================================================================
// Vec3 / Ray (math.hpp)
// ============================================================================

static void test_vec3_math() {
    // Basic arithmetic
    Vec3 a{1,2,3}, b{4,5,6};
    CHECK(near(a + b, {5,7,9}));
    CHECK(near(a - b, {-3,-3,-3}));
    CHECK(near(a * b, {4,10,18}));
    CHECK(near(a * 2.0f, {2,4,6}));
    CHECK(near(a / 2.0f, {0.5f,1.0f,1.5f}));

    // dot / cross
    CHECK(near(dot({1,0,0},{0,1,0}), 0.0f));
    CHECK(near(dot({1,2,3},{4,5,6}), 32.0f));

    Vec3 z = cross({1,0,0},{0,1,0});
    CHECK(near(z, {0,0,1}));

    // cross anticommutative
    Vec3 ab = cross(a, b);
    Vec3 ba = cross(b, a);
    CHECK(near(ab.x, -ba.x) && near(ab.y, -ba.y) && near(ab.z, -ba.z));

    // length / normalize
    CHECK(near(length({3,4,0}), 5.0f));
    Vec3 n = normalize({3,0,0});
    CHECK(near(n, {1,0,0}));
    CHECK(near(length(normalize({1,2,3})), 1.0f));

    // operator[]
    Vec3 v{7,8,9};
    CHECK(near(v[0],7) && near(v[1],8) && near(v[2],9));

    // scalar * Vec3 commutative
    CHECK(near(2.0f * Vec3{1,2,3}, {2,4,6}));
}

// ============================================================================
// RNG (rng.hpp)
// ============================================================================

static void test_rng() {
    // PCG32: all values in [0, 1)
    RNG rng(42u);
    bool allInRange = true;
    bool notAllSame = false;
    float prev = rng.nextFloat();
    for (int i = 0; i < 1000; ++i) {
        float f = rng.nextFloat();
        if (f < 0.0f || f >= 1.0f) allInRange = false;
        if (f != prev) notAllSame = true;
        prev = f;
    }
    CHECK(allInRange);
    CHECK(notAllSame);

    // HashRNG: all values in [0, 1)
    HashRNG hrng(0u);
    bool hAllInRange = true;
    bool hNotAllSame = false;
    float hprev = hrng.nextFloat();
    for (int i = 0; i < 1000; ++i) {
        float f = hrng.nextFloat();
        if (f < 0.0f || f >= 1.0f) hAllInRange = false;
        if (f != hprev) hNotAllSame = true;
        hprev = f;
    }
    CHECK(hAllInRange);
    CHECK(hNotAllSame);
}

// ============================================================================
// Hit default state (hit.hpp)
// ============================================================================

static void test_hit_defaults() {
    Hit h;
    CHECK(!h.hit);
    CHECK(h.t > 1e20f);
    CHECK(h.matId == 0);
    CHECK(h.sphereIdx == -1);
}

// ============================================================================
// Triangle intersection (triangle.hpp)
// ============================================================================

static void test_triangle() {
    // Unit triangle in XY-plane at z=0, hit from z=1 downward
    Triangle tri;
    tri.v[0] = {0,0,0};
    tri.v[1] = {1,0,0};
    tri.v[2] = {0,1,0};
    // flat face normals
    tri.n[0] = tri.n[1] = tri.n[2] = {0,0,1};

    Ray ray{{0.25f, 0.25f, 1.0f}, {0,0,-1}};
    float t, u, v;

    // Should hit
    bool hit = intersectTriangle(ray, tri, t, u, v);
    CHECK(hit);
    CHECK(near(t, 1.0f));
    // u, v ≥ 0 and u+v ≤ 1
    CHECK(u >= 0.0f && v >= 0.0f && u + v <= 1.0f + EPS);

    // Ray parallel to triangle — should miss
    Ray parallel{{0.25f, 0.25f, 1.0f}, {1,0,0}};
    CHECK(!intersectTriangle(parallel, tri, t, u, v));

    // Ray pointing away — should miss
    Ray away{{0.25f, 0.25f, 1.0f}, {0,0,1}};
    CHECK(!intersectTriangle(away, tri, t, u, v));

    // Ray outside triangle boundaries — should miss
    Ray outside{{2.0f, 2.0f, 1.0f}, {0,0,-1}};
    CHECK(!intersectTriangle(outside, tri, t, u, v));
}

// ============================================================================
// Sphere intersection (sphere.hpp)
// ============================================================================

static void test_sphere() {
    Sphere sph;
    sph.center = {0,0,0};
    sph.radius = 1.0f;
    sph.matId  = 0;
    sph.raymarch = false;

    // Direct hit: ray from (0,0,3) toward origin
    {
        Ray ray{{0,0,3},{0,0,-1}};
        Hit best;
        intersectSphere(ray, sph, best, 0);
        CHECK(best.hit);
        CHECK(near(best.t, 2.0f));              // hits front at z=1
        CHECK(near(length(best.n), 1.0f));      // normal is unit
        CHECK(best.n.z > 0.0f);                 // points toward ray origin
    }

    // Miss: ray parallel, offset by 2 units
    {
        Ray ray{{2,0,0},{0,0,-1}};
        Hit best;
        intersectSphere(ray, sph, best, 0);
        CHECK(!best.hit);
    }

    // Miss: ray pointing away
    {
        Ray ray{{0,0,3},{0,0,1}};
        Hit best;
        intersectSphere(ray, sph, best, 0);
        CHECK(!best.hit);
    }

    // Hit: normal at north pole is (0,1,0)
    {
        Ray ray{{0,3,0},{0,-1,0}};
        Hit best;
        intersectSphere(ray, sph, best, 0);
        CHECK(best.hit);
        CHECK(near(best.n, {0,1,0}, 1e-3f));
    }
}

// ============================================================================
// Plane intersection (plane.hpp)
// ============================================================================

static void test_plane() {
    Plane pl;
    pl.normal = {0,1,0};   // Y-up
    pl.offset = 0.0f;      // y=0

    // Hit from above
    {
        Ray ray{{0,2,0},{0,-1,0}};
        Hit best;
        intersectPlane(ray, pl, best);
        CHECK(best.hit);
        CHECK(near(best.t, 2.0f));
        CHECK(best.n.y > 0.0f);   // normal faces ray origin
    }

    // Miss: ray parallel to plane
    {
        Ray ray{{0,1,0},{1,0,0}};
        Hit best;
        intersectPlane(ray, pl, best);
        CHECK(!best.hit);
    }

    // Miss: ray pointing away (t would be negative)
    {
        Ray ray{{0,2,0},{0,1,0}};
        Hit best;
        intersectPlane(ray, pl, best);
        CHECK(!best.hit);
    }
}

// ============================================================================
// AABB (bvh.hpp)
// ============================================================================

static void test_aabb() {
    AABB box;
    box.expand({-1,-1,-1});
    box.expand({ 1, 1, 1});

    // Hit: ray from outside along x axis
    {
        Ray ray{{3,0,0},{-1,0,0}};
        CHECK(box.intersect(ray, 0.0f, 1e30f));
    }

    // Hit: ray from inside
    {
        Ray ray{{0,0,0},{1,0,0}};
        CHECK(box.intersect(ray, 0.0f, 1e30f));
    }

    // Miss: ray passing beside the box
    {
        Ray ray{{3,2,0},{-1,0,0}};
        CHECK(!box.intersect(ray, 0.0f, 1e30f));
    }

    // Miss: tMax too small (box too far)
    {
        Ray ray{{3,0,0},{-1,0,0}};
        CHECK(!box.intersect(ray, 0.0f, 1.5f));  // box starts at t=2
    }
}

// ============================================================================
// Sky gradient (tracer.hpp)
// ============================================================================

static void test_sky() {
    // Straight up → bluish
    Vec3 up = sky(normalize({0,1,0}));
    CHECK(up.z > up.x);   // more blue than red

    // Straight down → white-ish (t=0.5*(−1+1)=0 → all 1)
    Vec3 down = sky({0,-1,0});
    CHECK(near(down.x, 1.0f) && near(down.y, 1.0f) && near(down.z, 1.0f));

    // All components in [0,1]
    auto checkSkyRange = [](Vec3 d) {
        Vec3 c = sky(normalize(d));
        return c.x >= 0.0f && c.x <= 1.0f &&
               c.y >= 0.0f && c.y <= 1.0f &&
               c.z >= 0.0f && c.z <= 1.0f;
    };
    CHECK(checkSkyRange({1,0,0}));
    CHECK(checkSkyRange({0,0,1}));
    CHECK(checkSkyRange({1,1,1}));
}

// ============================================================================
// Cosine hemisphere sampler (tracer.hpp)
// ============================================================================

static void test_cosine_hemisphere() {
    // All samples should have z >= 0 and be (approximately) unit length
    bool allUpperHemisphere = true;
    bool allUnitLength      = true;
    for (int i = 0; i < 64; ++i) {
        float u1 = (float)i / 64.0f;
        float u2 = (float)(i * 37 % 64) / 64.0f;
        Vec3 s = sampleCosineHemisphere(u1, u2);
        if (s.z < -EPS)          allUpperHemisphere = false;
        if (!near(length(s), 1.0f, 1e-3f)) allUnitLength = false;
    }
    CHECK(allUpperHemisphere);
    CHECK(allUnitLength);
}

// ============================================================================
// ONB (tracer.hpp)
// ============================================================================

static void test_make_onb() {
    auto testONB = [](Vec3 n) -> bool {
        n = normalize(n);
        Vec3 t, b;
        makeONB(n, t, b);
        // All unit length
        if (!near(length(t), 1.0f, 1e-3f)) return false;
        if (!near(length(b), 1.0f, 1e-3f)) return false;
        // All orthogonal
        if (!near(dot(t, n), 0.0f, 1e-3f)) return false;
        if (!near(dot(b, n), 0.0f, 1e-3f)) return false;
        if (!near(dot(t, b), 0.0f, 1e-3f)) return false;
        return true;
    };

    CHECK(testONB({0,1,0}));
    CHECK(testONB({0,0,1}));
    CHECK(testONB({1,0,0}));
    CHECK(testONB({1,2,3}));
    CHECK(testONB({0,-1,0}));
}

// ============================================================================
// Fresnel / GGX helpers (tracer.hpp)
// ============================================================================

static void test_brdf_helpers() {
    // fresnelSchlick: cosTheta=0 (grazing) → 1.0, cosTheta=1 (normal) → F0
    Vec3 F0{0.04f, 0.04f, 0.04f};
    Vec3 fGrazing = fresnelSchlick(0.0f, F0);  // grazing: returns 1
    Vec3 fNormal  = fresnelSchlick(1.0f, F0);  // normal incidence: returns F0
    CHECK(near(fGrazing.x, 1.0f, 1e-3f));
    CHECK(near(fNormal.x, F0.x, 1e-3f));

    // D_GGX: positive, peaks at NoH=1
    float d1 = D_GGX(1.0f, 0.5f);  // NoH=1 (perfect alignment)
    float d2 = D_GGX(0.5f, 0.5f);  // NoH=0.5 (off-axis)
    CHECK(d1 > 0.0f);
    CHECK(d1 > d2);

    // G1_GGX: ∈ (0, 1] for NdotV > 0
    float g = G1_GGX(0.5f, 0.5f);
    CHECK(g > 0.0f && g <= 1.0f + EPS);

    // G2 ≤ G1 for the same angle
    float g1 = G1_GGX(0.7f, 0.3f);
    float g2 = G2_GGX(0.7f, 0.7f, 0.3f);
    CHECK(g2 <= g1 + EPS);

    // reflect: reflect (0,0,-1) about (0,0,1) → (0,0,1)
    Vec3 inc{0,0,-1}, nrm{0,0,1};
    Vec3 ref = reflect(inc, nrm);
    CHECK(near(ref, {0,0,1}, 1e-3f));

    // misPower: complementary weights (pdf equal → both 0.5)
    float w1 = misPower(1.0f, 1.0f);
    CHECK(near(w1, 0.5f, 1e-3f));
    // larger pdfA → larger weight
    float wa = misPower(2.0f, 1.0f);
    float wb = misPower(1.0f, 2.0f);
    CHECK(wa > wb);
}

// ============================================================================
// Camera ray generation (camera.hpp)
// ============================================================================

static void test_camera() {
    Camera cam;
    cam.setLookAt({0,0,5}, {0,0,0});

    // Center pixel ray should point roughly in the -z direction
    Ray r = cam.generateRay(320, 240, 640, 480, 0.5f, 0.5f);
    CHECK(r.d.z < -0.9f);  // mostly forward
    CHECK(near(length(r.d), 1.0f, 1e-3f));

    // Ray origin should be at eye
    CHECK(near(r.o, {0,0,5}, 1e-3f));

    // Top-left vs bottom-right pixels should produce diverging rays
    Ray tl = cam.generateRay(  0,   0, 640, 480, 0.5f, 0.5f);
    Ray br = cam.generateRay(639, 479, 640, 480, 0.5f, 0.5f);
    CHECK(tl.d.x < br.d.x);  // tl goes left, br goes right
    CHECK(tl.d.y > br.d.y);  // tl goes up, br goes down
}

// ============================================================================
// tracePath: empty scene → returns sky color
// ============================================================================

static void test_tracepath_empty_scene() {
    PBRScene scene;
    scene.materials.push_back(Material{});  // mat 0

    RNG rng(1234u);

    // Ray pointing straight up → bluish sky
    Ray ray{{0,0,0},{0,1,0}};
    Vec3 L = tracePath(ray, rng, scene.materials, scene);

    // Result must be finite and non-negative
    CHECK(std::isfinite(L.x) && std::isfinite(L.y) && std::isfinite(L.z));
    CHECK(L.x >= 0.0f && L.y >= 0.0f && L.z >= 0.0f);

    // Should match sky directly (no geometry)
    Vec3 expected = sky({0,1,0});
    CHECK(near(L.x, expected.x, 1e-3f));
    CHECK(near(L.y, expected.y, 1e-3f));
    CHECK(near(L.z, expected.z, 1e-3f));
}

// ============================================================================
// Main
// ============================================================================

int main() {
    test_vec3_math();
    test_rng();
    test_hit_defaults();
    test_triangle();
    test_sphere();
    test_plane();
    test_aabb();
    test_sky();
    test_cosine_hemisphere();
    test_make_onb();
    test_brdf_helpers();
    test_camera();
    test_tracepath_empty_scene();

    if (failures == 0) {
        std::printf("\nAll tests passed.\n");
        return 0;
    } else {
        std::fprintf(stderr, "\n%d test(s) FAILED.\n", failures);
        return 1;
    }
}
