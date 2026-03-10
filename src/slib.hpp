#pragma once
#include <array>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>
#include "texture.hpp"

namespace slib
{
    struct vec2;
    struct vec3;
    struct vec4;
    struct mat4;

    struct zvec2
    {
        float x, y, w;
        constexpr zvec2() : x(0.0f), y(0.0f), w(1.0f) {}
        constexpr zvec2(float _x, float _y, float _w = 1.0f) : x(_x), y(_y), w(_w) {}
        zvec2& operator*=(const zvec2& rhs);
        zvec2& operator*=(float rhs);
        zvec2& operator/=(float rhs);
        zvec2 operator-(const zvec2& rhs) const;
        zvec2 operator+(const zvec2& rhs) const;
        zvec2 operator*(float rhs) const;
        zvec2 operator/(float rhs) const;
        zvec2& operator+=(const zvec2& rhs);
    };

    struct vec2
    {
        float x, y;
        constexpr vec2() : x(0.0f), y(0.0f) {}
        constexpr vec2(float _x, float _y) : x(_x), y(_y) {}
        vec2& operator*=(const vec2& rhs);
        vec2& operator*=(float rhs);
        vec2 operator-(const vec2& rhs) const;
        vec2 operator+(const vec2& rhs) const;
    };

    struct vec3
    {
        float x, y, z;
        constexpr vec3() : x(0.0f), y(0.0f), z(0.0f) {}
        constexpr vec3(float _x, float _y, float _z) : x(_x), y(_y), z(_z) {}
        vec3& operator+=(float rhs);
        vec3& operator-=(float rhs);
        vec3& operator*=(float rhs);
        vec3& operator/=(float rhs);
        vec3 operator-(float rhs) const;
        vec3 operator+(float rhs) const;
        vec3 operator*(float rhs) const;
        vec3 operator/(float rhs) const;
        vec3& operator+=(const vec3& rhs);
        vec3& operator-=(const vec3& rhs);
        vec3& operator*=(const vec3& rhs);
        vec3& operator/=(const vec3& rhs);
        vec3 operator-(const vec3& rhs) const;
        vec3 operator+(const vec3& rhs) const;
        vec3 operator*(const vec3& rhs) const;
        vec3 operator/(const vec3& rhs) const;
        bool operator==(const vec3& rhs) const;
        bool operator==(float rhs) const;
        bool operator<(const vec3& rhs) const;
        bool operator>(const vec3& rhs) const;
        bool operator<=(const vec3& rhs) const;
        bool operator>=(const vec3& rhs) const;
        vec3& operator=(const vec4& rhs);

    };

    struct vec4
    {
        float x, y, z, w;
        vec4 operator*(const mat4& rhs) const;
        vec4 operator*=(const mat4& rhs);
        vec4 operator-(const vec4& rhs) const;
        vec4 operator+(const vec4& rhs) const;
        vec4& operator+=(const vec4& rhs);
        vec4& operator-=(const vec4& rhs);
        vec4 operator*(float rhs) const;
        vec4 operator/(float rhs) const;
        vec4() = default;
        vec4(const vec3& v3, const float _w) : x(v3.x), y(v3.y), z(v3.z), w(_w)
        {
        }
        vec4(const float _x, const float _y, const float _z, const float _w) : x(_x), y(_y), z(_z), w(_w)
        {
        }
    };

    struct mat4
    {
        float data[16];
        mat4() = default;
        mat4(std::initializer_list<std::initializer_list<float>> rows);

        mat4& operator+=(const mat4& rhs);
        mat4& operator*=(const mat4& rhs);
        mat4 operator*(const mat4& rhs) const;
        vec4 operator*(const vec4& rhs) const;
    };

} // namespace slib
