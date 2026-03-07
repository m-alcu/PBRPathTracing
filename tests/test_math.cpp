#include <cmath>
#include <cstdio>
#include <vector>
#include "../src/slib.hpp"
#include "../src/smath.hpp"

constexpr float EPSILON = 1e-5f;

static int failures = 0;

static bool nearEqual(float a, float b, float eps = EPSILON) {
    return std::abs(a - b) < eps;
}
static bool nearEqual(const slib::vec3& a, const slib::vec3& b, float eps = EPSILON) {
    return nearEqual(a.x, b.x, eps) && nearEqual(a.y, b.y, eps) && nearEqual(a.z, b.z, eps);
}
static bool nearEqual(const slib::vec4& a, const slib::vec4& b, float eps = EPSILON) {
    return nearEqual(a.x, b.x, eps) && nearEqual(a.y, b.y, eps) &&
           nearEqual(a.z, b.z, eps) && nearEqual(a.w, b.w, eps);
}

#define CHECK(expr) do { \
    if (!(expr)) { std::fprintf(stderr, "FAIL [%s:%d]: %s\n", __FILE__, __LINE__, #expr); ++failures; } \
    else { std::printf("PASS: %s\n", #expr); } \
} while(0)

// ============================================================================
// vec2
// ============================================================================

static void test_vec2() {
    // Addition
    slib::vec2 a{1.0f, 2.0f}, b{3.0f, 4.0f};
    auto r = a + b;
    CHECK(nearEqual(r.x, 4.0f) && nearEqual(r.y, 6.0f));

    // Subtraction
    slib::vec2 c{5.0f, 7.0f}, d{2.0f, 3.0f};
    auto s = c - d;
    CHECK(nearEqual(s.x, 3.0f) && nearEqual(s.y, 4.0f));

    // Scalar multiplication
    slib::vec2 e{2.0f, 3.0f};
    e *= 2.0f;
    CHECK(nearEqual(e.x, 4.0f) && nearEqual(e.y, 6.0f));
}

// ============================================================================
// vec3
// ============================================================================

static void test_vec3() {
    // Addition
    {
        slib::vec3 a{1,2,3}, b{4,5,6};
        auto r = a + b;
        CHECK(nearEqual(r.x,5) && nearEqual(r.y,7) && nearEqual(r.z,9));
    }
    // Subtraction
    {
        slib::vec3 a{5,7,9}, b{1,2,3};
        auto r = a - b;
        CHECK(nearEqual(r.x,4) && nearEqual(r.y,5) && nearEqual(r.z,6));
    }
    // Scalar mul
    {
        slib::vec3 a{1,2,3};
        auto r = a * 2.0f;
        CHECK(nearEqual(r.x,2) && nearEqual(r.y,4) && nearEqual(r.z,6));
    }
    // Scalar div
    {
        slib::vec3 a{4,6,8};
        auto r = a / 2.0f;
        CHECK(nearEqual(r.x,2) && nearEqual(r.y,3) && nearEqual(r.z,4));
    }
    // Component-wise mul
    {
        slib::vec3 a{2,3,4}, b{5,6,7};
        auto r = a * b;
        CHECK(nearEqual(r.x,10) && nearEqual(r.y,18) && nearEqual(r.z,28));
    }
    // Compound addition
    {
        slib::vec3 a{1,2,3};
        a += slib::vec3{4,5,6};
        CHECK(nearEqual(a.x,5) && nearEqual(a.y,7) && nearEqual(a.z,9));
    }
    // Equality
    {
        slib::vec3 a{1,2,3}, b{1,2,3}, c{1,2,4};
        CHECK(a == b);
        CHECK(!(a == c));
    }
    // Equality with epsilon
    {
        slib::vec3 a{1,2,3};
        slib::vec3 b{1+1e-7f, 2-1e-7f, 3+1e-7f};
        CHECK(a == b);

        slib::vec3 unit{1,0,0};
        slib::vec3 norm = smath::normalize({5,0,0});
        CHECK(unit == norm);

        slib::vec3 almostZero{1e-7f, -1e-7f, 1e-7f};
        CHECK(almostZero == 0.0f);
    }
}

// ============================================================================
// vec4
// ============================================================================

static void test_vec4() {
    // Constructor from vec3
    {
        slib::vec3 v3{1,2,3};
        slib::vec4 v4(v3, 1.0f);
        CHECK(nearEqual(v4.x,1) && nearEqual(v4.y,2) && nearEqual(v4.z,3) && nearEqual(v4.w,1));
    }
    // Addition
    {
        slib::vec4 a{1,2,3,4}, b{5,6,7,8};
        auto r = a + b;
        CHECK(nearEqual(r.x,6) && nearEqual(r.y,8) && nearEqual(r.z,10) && nearEqual(r.w,12));
    }
    // Scalar mul
    {
        slib::vec4 a{1,2,3,4};
        auto r = a * 2.0f;
        CHECK(nearEqual(r.x,2) && nearEqual(r.y,4) && nearEqual(r.z,6) && nearEqual(r.w,8));
    }
}

// ============================================================================
// mat4
// ============================================================================

static void test_mat4() {
    // Identity diagonal/off-diagonal
    {
        slib::mat4 I = smath::identity();
        CHECK(nearEqual(I.data[0],1) && nearEqual(I.data[5],1) &&
              nearEqual(I.data[10],1) && nearEqual(I.data[15],1));
        CHECK(nearEqual(I.data[1],0) && nearEqual(I.data[2],0) && nearEqual(I.data[4],0));
    }
    // Identity * vector
    {
        slib::mat4 I = smath::identity();
        slib::vec4 v{1,2,3,1};
        auto r = I * v;
        CHECK(nearEqual(r.x,1) && nearEqual(r.y,2) && nearEqual(r.z,3) && nearEqual(r.w,1));
    }
    // Identity * Identity
    {
        slib::mat4 r = smath::identity() * smath::identity();
        CHECK(nearEqual(r.data[0],1) && nearEqual(r.data[5],1) &&
              nearEqual(r.data[10],1) && nearEqual(r.data[15],1));
    }
}

// ============================================================================
// smath functions
// ============================================================================

static void test_smath() {
    // distance (magnitude)
    CHECK(nearEqual(smath::distance({3,4,0}), 5.0f));

    // normalize
    {
        auto n = smath::normalize({3,0,0});
        CHECK(nearEqual(n.x,1) && nearEqual(n.y,0) && nearEqual(n.z,0));
    }
    // normalize → unit length
    {
        auto n = smath::normalize({1,2,2});
        CHECK(nearEqual(smath::distance(n), 1.0f));
    }
    // normalize zero vector
    {
        auto r = smath::normalize({0,0,0});
        CHECK(nearEqual(r.x,0) && nearEqual(r.y,0) && nearEqual(r.z,0));
    }
    // normalize near-zero vector
    {
        auto r = smath::normalize({1e-8f,1e-8f,1e-8f});
        CHECK(nearEqual(r.x,0) && nearEqual(r.y,0) && nearEqual(r.z,0));
    }
    // dot product
    {
        slib::vec3 x{1,0,0}, y{0,1,0};
        CHECK(nearEqual(smath::dot(x,y), 0.0f));
        CHECK(nearEqual(smath::dot(x,x), 1.0f));
        CHECK(nearEqual(smath::dot({1,2,3},{4,5,6}), 32.0f));
    }
    // cross product
    {
        auto z = smath::cross({1,0,0},{0,1,0});
        CHECK(nearEqual(z.x,0) && nearEqual(z.y,0) && nearEqual(z.z,1));
    }
    // cross anticommutative
    {
        slib::vec3 a{1,2,3}, b{4,5,6};
        auto ab = smath::cross(a,b);
        auto ba = smath::cross(b,a);
        CHECK(nearEqual(ab.x,-ba.x) && nearEqual(ab.y,-ba.y) && nearEqual(ab.z,-ba.z));
    }
    // centroid
    {
        std::vector<slib::vec3> pts = {{0,0,0},{2,0,0},{2,2,0},{0,2,0}};
        auto c = smath::centroid(pts);
        CHECK(nearEqual(c.x,1) && nearEqual(c.y,1) && nearEqual(c.z,0));
    }
    // translation matrix
    {
        slib::mat4 t = smath::translation({5,10,15});
        auto r = t * slib::vec4{0,0,0,1};
        CHECK(nearEqual(r.x,5) && nearEqual(r.y,10) && nearEqual(r.z,15) && nearEqual(r.w,1));
    }
    // scale matrix
    {
        slib::mat4 s = smath::scale({2,3,4});
        auto r = s * slib::vec4{1,1,1,1};
        CHECK(nearEqual(r.x,2) && nearEqual(r.y,3) && nearEqual(r.z,4));
    }
    // rotation identity
    {
        slib::mat4 r = smath::rotation({0,0,0});
        auto v = r * slib::vec4{1,0,0,1};
        CHECK(nearEqual(v.x,1,EPSILON) && nearEqual(v.y,0,EPSILON) && nearEqual(v.z,0,EPSILON));
    }
    // rotation 90° around Z
    {
        slib::mat4 r = smath::rotation({0,0,90});
        auto v = r * slib::vec4{1,0,0,1};
        CHECK(nearEqual(v.x,0,EPSILON) && nearEqual(v.y,1,EPSILON) && nearEqual(v.z,0,EPSILON));
    }
}

// ============================================================================
// Main
// ============================================================================

int main() {
    test_vec2();
    test_vec3();
    test_vec4();
    test_mat4();
    test_smath();

    if (failures == 0) {
        std::printf("\nAll tests passed.\n");
        return 0;
    } else {
        std::fprintf(stderr, "\n%d test(s) FAILED.\n", failures);
        return 1;
    }
}
