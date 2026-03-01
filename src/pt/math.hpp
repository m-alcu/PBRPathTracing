#pragma once
#include <cmath>
#include <algorithm>

struct Vec3 {
    float x = 0, y = 0, z = 0;
    Vec3() = default;
    Vec3(float x, float y, float z) : x(x), y(y), z(z) {}

    Vec3 operator+(const Vec3& o) const { return {x+o.x, y+o.y, z+o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x-o.x, y-o.y, z-o.z}; }
    Vec3 operator*(const Vec3& o) const { return {x*o.x, y*o.y, z*o.z}; }
    Vec3 operator*(float t)        const { return {x*t, y*t, z*t}; }
    Vec3 operator/(float t)        const { return {x/t, y/t, z/t}; }
    Vec3& operator+=(const Vec3& o)  { x+=o.x; y+=o.y; z+=o.z; return *this; }
    Vec3& operator*=(const Vec3& o)  { x*=o.x; y*=o.y; z*=o.z; return *this; }
    Vec3& operator*=(float t)        { x*=t; y*=t; z*=t; return *this; }
    float operator[](int i) const    { return i==0 ? x : (i==1 ? y : z); }
};

inline Vec3 operator*(float t, const Vec3& v) { return v * t; }

inline float dot(const Vec3& a, const Vec3& b) {
    return a.x*b.x + a.y*b.y + a.z*b.z;
}
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline float length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline Vec3 normalize(const Vec3& v) { return v / length(v); }

struct Ray { Vec3 o, d; };
