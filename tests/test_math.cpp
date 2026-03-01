#include <gtest/gtest.h>
#include <cmath>
#include "../src/slib.hpp"
#include "../src/smath.hpp"

// Helper for floating-point comparisons
constexpr float EPSILON = 1e-5f;

bool nearEqual(float a, float b, float eps = EPSILON) {
    return std::abs(a - b) < eps;
}

bool nearEqual(const slib::vec3& a, const slib::vec3& b, float eps = EPSILON) {
    return nearEqual(a.x, b.x, eps) && nearEqual(a.y, b.y, eps) && nearEqual(a.z, b.z, eps);
}

bool nearEqual(const slib::vec4& a, const slib::vec4& b, float eps = EPSILON) {
    return nearEqual(a.x, b.x, eps) && nearEqual(a.y, b.y, eps) &&
           nearEqual(a.z, b.z, eps) && nearEqual(a.w, b.w, eps);
}

// ============================================================================
// vec2 Tests
// ============================================================================

TEST(Vec2Test, Addition) {
    slib::vec2 a{1.0f, 2.0f};
    slib::vec2 b{3.0f, 4.0f};
    slib::vec2 result = a + b;

    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 6.0f);
}

TEST(Vec2Test, Subtraction) {
    slib::vec2 a{5.0f, 7.0f};
    slib::vec2 b{2.0f, 3.0f};
    slib::vec2 result = a - b;

    EXPECT_FLOAT_EQ(result.x, 3.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
}

TEST(Vec2Test, ScalarMultiplication) {
    slib::vec2 a{2.0f, 3.0f};
    a *= 2.0f;

    EXPECT_FLOAT_EQ(a.x, 4.0f);
    EXPECT_FLOAT_EQ(a.y, 6.0f);
}

// ============================================================================
// vec3 Tests
// ============================================================================

TEST(Vec3Test, Addition) {
    slib::vec3 a{1.0f, 2.0f, 3.0f};
    slib::vec3 b{4.0f, 5.0f, 6.0f};
    slib::vec3 result = a + b;

    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 7.0f);
    EXPECT_FLOAT_EQ(result.z, 9.0f);
}

TEST(Vec3Test, Subtraction) {
    slib::vec3 a{5.0f, 7.0f, 9.0f};
    slib::vec3 b{1.0f, 2.0f, 3.0f};
    slib::vec3 result = a - b;

    EXPECT_FLOAT_EQ(result.x, 4.0f);
    EXPECT_FLOAT_EQ(result.y, 5.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST(Vec3Test, ScalarMultiplication) {
    slib::vec3 a{1.0f, 2.0f, 3.0f};
    slib::vec3 result = a * 2.0f;

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
}

TEST(Vec3Test, ScalarDivision) {
    slib::vec3 a{4.0f, 6.0f, 8.0f};
    slib::vec3 result = a / 2.0f;

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 4.0f);
}

TEST(Vec3Test, ComponentWiseMultiplication) {
    slib::vec3 a{2.0f, 3.0f, 4.0f};
    slib::vec3 b{5.0f, 6.0f, 7.0f};
    slib::vec3 result = a * b;

    EXPECT_FLOAT_EQ(result.x, 10.0f);
    EXPECT_FLOAT_EQ(result.y, 18.0f);
    EXPECT_FLOAT_EQ(result.z, 28.0f);
}

TEST(Vec3Test, CompoundAddition) {
    slib::vec3 a{1.0f, 2.0f, 3.0f};
    a += slib::vec3{4.0f, 5.0f, 6.0f};

    EXPECT_FLOAT_EQ(a.x, 5.0f);
    EXPECT_FLOAT_EQ(a.y, 7.0f);
    EXPECT_FLOAT_EQ(a.z, 9.0f);
}

TEST(Vec3Test, Equality) {
    slib::vec3 a{1.0f, 2.0f, 3.0f};
    slib::vec3 b{1.0f, 2.0f, 3.0f};
    slib::vec3 c{1.0f, 2.0f, 4.0f};

    EXPECT_TRUE(a == b);
    EXPECT_FALSE(a == c);
}

TEST(Vec3Test, EqualityWithEpsilon) {
    // Values within epsilon should be equal
    slib::vec3 a{1.0f, 2.0f, 3.0f};
    slib::vec3 b{1.0f + 1e-7f, 2.0f - 1e-7f, 3.0f + 1e-7f};
    EXPECT_TRUE(a == b);

    // Normalized vector should equal unit vector
    slib::vec3 unit{1.0f, 0.0f, 0.0f};
    slib::vec3 normalized = smath::normalize({5.0f, 0.0f, 0.0f});
    EXPECT_TRUE(unit == normalized);

    // Scalar comparison with epsilon
    slib::vec3 almostZero{1e-7f, -1e-7f, 1e-7f};
    EXPECT_TRUE(almostZero == 0.0f);
}

// ============================================================================
// vec4 Tests
// ============================================================================

TEST(Vec4Test, ConstructorFromVec3) {
    slib::vec3 v3{1.0f, 2.0f, 3.0f};
    slib::vec4 v4(v3, 1.0f);

    EXPECT_FLOAT_EQ(v4.x, 1.0f);
    EXPECT_FLOAT_EQ(v4.y, 2.0f);
    EXPECT_FLOAT_EQ(v4.z, 3.0f);
    EXPECT_FLOAT_EQ(v4.w, 1.0f);
}

TEST(Vec4Test, Addition) {
    slib::vec4 a{1.0f, 2.0f, 3.0f, 4.0f};
    slib::vec4 b{5.0f, 6.0f, 7.0f, 8.0f};
    slib::vec4 result = a + b;

    EXPECT_FLOAT_EQ(result.x, 6.0f);
    EXPECT_FLOAT_EQ(result.y, 8.0f);
    EXPECT_FLOAT_EQ(result.z, 10.0f);
    EXPECT_FLOAT_EQ(result.w, 12.0f);
}

TEST(Vec4Test, ScalarMultiplication) {
    slib::vec4 a{1.0f, 2.0f, 3.0f, 4.0f};
    slib::vec4 result = a * 2.0f;

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 4.0f);
    EXPECT_FLOAT_EQ(result.z, 6.0f);
    EXPECT_FLOAT_EQ(result.w, 8.0f);
}

// ============================================================================
// mat4 Tests
// ============================================================================

TEST(Mat4Test, IdentityMatrix) {
    slib::mat4 identity = smath::identity();

    // Diagonal should be 1
    EXPECT_FLOAT_EQ(identity.data[0],  1.0f);
    EXPECT_FLOAT_EQ(identity.data[5],  1.0f);
    EXPECT_FLOAT_EQ(identity.data[10], 1.0f);
    EXPECT_FLOAT_EQ(identity.data[15], 1.0f);

    // Off-diagonal should be 0
    EXPECT_FLOAT_EQ(identity.data[1], 0.0f);
    EXPECT_FLOAT_EQ(identity.data[2], 0.0f);
    EXPECT_FLOAT_EQ(identity.data[4], 0.0f);
}

TEST(Mat4Test, IdentityTimesVector) {
    slib::mat4 identity = smath::identity();
    slib::vec4 v{1.0f, 2.0f, 3.0f, 1.0f};

    slib::vec4 result = identity * v;

    EXPECT_FLOAT_EQ(result.x, 1.0f);
    EXPECT_FLOAT_EQ(result.y, 2.0f);
    EXPECT_FLOAT_EQ(result.z, 3.0f);
    EXPECT_FLOAT_EQ(result.w, 1.0f);
}

TEST(Mat4Test, MatrixMultiplication) {
    slib::mat4 a = smath::identity();
    slib::mat4 b = smath::identity();

    // Identity * Identity = Identity
    slib::mat4 result = a * b;

    EXPECT_FLOAT_EQ(result.data[0],  1.0f);
    EXPECT_FLOAT_EQ(result.data[5],  1.0f);
    EXPECT_FLOAT_EQ(result.data[10], 1.0f);
    EXPECT_FLOAT_EQ(result.data[15], 1.0f);
}

// ============================================================================
// smath Function Tests
// ============================================================================

TEST(SmathTest, Distance) {
    slib::vec3 v{3.0f, 4.0f, 0.0f};
    float d = smath::distance(v);

    EXPECT_FLOAT_EQ(d, 5.0f);  // 3-4-5 triangle
}

TEST(SmathTest, Normalize) {
    slib::vec3 v{3.0f, 0.0f, 0.0f};
    slib::vec3 n = smath::normalize(v);

    EXPECT_FLOAT_EQ(n.x, 1.0f);
    EXPECT_FLOAT_EQ(n.y, 0.0f);
    EXPECT_FLOAT_EQ(n.z, 0.0f);
}

TEST(SmathTest, NormalizeUnitLength) {
    slib::vec3 v{1.0f, 2.0f, 2.0f};  // length = 3
    slib::vec3 n = smath::normalize(v);

    float length = smath::distance(n);
    EXPECT_NEAR(length, 1.0f, EPSILON);
}

TEST(SmathTest, NormalizeZeroVector) {
    // Normalizing zero vector should return zero vector (not crash)
    slib::vec3 zero{0.0f, 0.0f, 0.0f};
    slib::vec3 result = smath::normalize(zero);

    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST(SmathTest, NormalizeNearZeroVector) {
    // Very small vector should also return zero (avoid division by tiny number)
    slib::vec3 tiny{1e-8f, 1e-8f, 1e-8f};
    slib::vec3 result = smath::normalize(tiny);

    EXPECT_FLOAT_EQ(result.x, 0.0f);
    EXPECT_FLOAT_EQ(result.y, 0.0f);
    EXPECT_FLOAT_EQ(result.z, 0.0f);
}

TEST(SmathTest, DotProduct) {
    slib::vec3 a{1.0f, 0.0f, 0.0f};
    slib::vec3 b{0.0f, 1.0f, 0.0f};

    // Perpendicular vectors have dot product of 0
    EXPECT_FLOAT_EQ(smath::dot(a, b), 0.0f);

    // Parallel vectors
    slib::vec3 c{1.0f, 0.0f, 0.0f};
    EXPECT_FLOAT_EQ(smath::dot(a, c), 1.0f);

    // General case
    slib::vec3 d{1.0f, 2.0f, 3.0f};
    slib::vec3 e{4.0f, 5.0f, 6.0f};
    EXPECT_FLOAT_EQ(smath::dot(d, e), 32.0f);  // 1*4 + 2*5 + 3*6 = 32
}

TEST(SmathTest, CrossProduct) {
    slib::vec3 x{1.0f, 0.0f, 0.0f};
    slib::vec3 y{0.0f, 1.0f, 0.0f};

    slib::vec3 z = smath::cross(x, y);

    EXPECT_FLOAT_EQ(z.x, 0.0f);
    EXPECT_FLOAT_EQ(z.y, 0.0f);
    EXPECT_FLOAT_EQ(z.z, 1.0f);
}

TEST(SmathTest, CrossProductAnticommutative) {
    slib::vec3 a{1.0f, 2.0f, 3.0f};
    slib::vec3 b{4.0f, 5.0f, 6.0f};

    slib::vec3 ab = smath::cross(a, b);
    slib::vec3 ba = smath::cross(b, a);

    // cross(a,b) = -cross(b,a)
    EXPECT_FLOAT_EQ(ab.x, -ba.x);
    EXPECT_FLOAT_EQ(ab.y, -ba.y);
    EXPECT_FLOAT_EQ(ab.z, -ba.z);
}

TEST(SmathTest, Centroid) {
    std::vector<slib::vec3> points = {
        {0.0f, 0.0f, 0.0f},
        {2.0f, 0.0f, 0.0f},
        {2.0f, 2.0f, 0.0f},
        {0.0f, 2.0f, 0.0f}
    };

    slib::vec3 c = smath::centroid(points);

    EXPECT_FLOAT_EQ(c.x, 1.0f);
    EXPECT_FLOAT_EQ(c.y, 1.0f);
    EXPECT_FLOAT_EQ(c.z, 0.0f);
}

TEST(SmathTest, TranslationMatrix) {
    slib::mat4 t = smath::translation({5.0f, 10.0f, 15.0f});
    slib::vec4 v{0.0f, 0.0f, 0.0f, 1.0f};

    slib::vec4 result = t * v;

    EXPECT_FLOAT_EQ(result.x, 5.0f);
    EXPECT_FLOAT_EQ(result.y, 10.0f);
    EXPECT_FLOAT_EQ(result.z, 15.0f);
    EXPECT_FLOAT_EQ(result.w, 1.0f);
}

TEST(SmathTest, ScaleMatrix) {
    slib::mat4 s = smath::scale({2.0f, 3.0f, 4.0f});
    slib::vec4 v{1.0f, 1.0f, 1.0f, 1.0f};

    slib::vec4 result = s * v;

    EXPECT_FLOAT_EQ(result.x, 2.0f);
    EXPECT_FLOAT_EQ(result.y, 3.0f);
    EXPECT_FLOAT_EQ(result.z, 4.0f);
}

TEST(SmathTest, RotationMatrixIdentity) {
    // No rotation
    slib::mat4 r = smath::rotation({0.0f, 0.0f, 0.0f});
    slib::vec4 v{1.0f, 0.0f, 0.0f, 1.0f};

    slib::vec4 result = r * v;

    EXPECT_NEAR(result.x, 1.0f, EPSILON);
    EXPECT_NEAR(result.y, 0.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
}

TEST(SmathTest, RotationMatrix90DegreesZ) {
    // 90 degree rotation around Z axis (rotation() expects degrees, not radians)
    slib::mat4 r = smath::rotation({0.0f, 0.0f, 90.0f});
    slib::vec4 v{1.0f, 0.0f, 0.0f, 1.0f};

    slib::vec4 result = r * v;

    // (1,0,0) rotated 90 degrees around Z should give (0,1,0)
    EXPECT_NEAR(result.x, 0.0f, EPSILON);
    EXPECT_NEAR(result.y, 1.0f, EPSILON);
    EXPECT_NEAR(result.z, 0.0f, EPSILON);
}

// ============================================================================
// Main
// ============================================================================

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
