#include "slib.hpp"
#include <cmath>
#include <iostream>
#include <limits>

namespace slib
{

    vec2& vec2::operator*=(const vec2& rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        return *this;
    }

    vec2& vec2::operator*=(const float rhs)
    {
        x *= rhs;
        y *= rhs;
        return *this;
    }

    vec2 vec2::operator-(const vec2& rhs) const
    {
        return {x - rhs.x, y - rhs.y};
    }

    vec2 vec2::operator+(const vec2& rhs) const
    {
        return {x + rhs.x, y + rhs.y};
    }

    zvec2& zvec2::operator*=(const zvec2& rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        w *= rhs.w;
        return *this;
    }

    zvec2& zvec2::operator*=(const float rhs)
    {
        x *= rhs;
        y *= rhs;
        w *= rhs;
        return *this;
    }

    zvec2& zvec2::operator/=(const float rhs)
    {
        x /= rhs;
        y /= rhs;
        w /= rhs;
        return *this;
    }

    zvec2& zvec2::operator+=(const zvec2& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        w += rhs.w;
        return *this;
    }

    zvec2 zvec2::operator-(const zvec2& rhs) const
    {
        return {x - rhs.x, y - rhs.y, w - rhs.w};
    }

    zvec2 zvec2::operator+(const zvec2& rhs) const
    {
        return {x + rhs.x, y + rhs.y, w + rhs.w};
    }

    zvec2 zvec2::operator*(const float rhs) const
    {
        return {x * rhs, y * rhs, w * rhs};
    }
    
    zvec2 zvec2::operator/(const float rhs) const
    {
        return {x / rhs, y / rhs, w / rhs};
    }


    vec3& vec3::operator+=(const float rhs)
    {
        x += rhs;
        y += rhs;
        z += rhs;
        return *this;
    }

    vec3& vec3::operator-=(const float rhs)
    {
        x -= rhs;
        y -= rhs;
        z -= rhs;
        return *this;
    }

    vec3& vec3::operator*=(const float rhs)
    {
        x *= rhs;
        y *= rhs;
        z *= rhs;
        return *this;
    }

    vec3& vec3::operator/=(const float rhs)
    {
        x /= rhs;
        y /= rhs;
        z /= rhs;
        return *this;
    }

    vec3 vec3::operator-(const float rhs) const
    {
        return {x - rhs, y - rhs, z - rhs};
    }

    vec3 vec3::operator+(const float rhs) const
    {
        return {x + rhs, y + rhs, z + rhs};
    }

    vec3 vec3::operator*(const float rhs) const
    {
        return {x * rhs, y * rhs, z * rhs};
    }

    vec3 vec3::operator/(const float rhs) const
    {
        return {x / rhs, y / rhs, z / rhs};
    }

    vec3& vec3::operator+=(const vec3& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        return *this;
    }

    vec3& vec3::operator-=(const vec3& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        return *this;
    }

    vec3& vec3::operator*=(const vec3& rhs)
    {
        x *= rhs.x;
        y *= rhs.y;
        z *= rhs.z;
        return *this;
    }

    vec3& vec3::operator/=(const vec3& rhs)
    {
        x /= rhs.x;
        y /= rhs.y;
        z /= rhs.z;
        return *this;
    }

    vec3 vec3::operator-(const vec3& rhs) const
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z};
    }

    vec3 vec3::operator+(const vec3& rhs) const
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z};
    }

    vec3 vec3::operator*(const vec3& rhs) const
    {
        return {x * rhs.x, y * rhs.y, z * rhs.z};
    }

    vec3 vec3::operator/(const vec3& rhs) const
    {
        return {x / rhs.x, y / rhs.y, z / rhs.z};
    }

    bool vec3::operator==(const vec3& rhs) const
    {
        constexpr float EPSILON = 1e-6f;
        return std::abs(x - rhs.x) < EPSILON &&
               std::abs(y - rhs.y) < EPSILON &&
               std::abs(z - rhs.z) < EPSILON;
    }

    bool vec3::operator==(float rhs) const
    {
        constexpr float EPSILON = 1e-6f;
        return std::abs(x - rhs) < EPSILON &&
               std::abs(y - rhs) < EPSILON &&
               std::abs(z - rhs) < EPSILON;
    }

    bool vec3::operator<(const vec3& rhs) const
    {
        return x < rhs.x && y < rhs.y && z < rhs.z;
    }

    bool vec3::operator>(const vec3& rhs) const
    {
        return !(*this < rhs);
    }

    bool vec3::operator<=(const vec3& rhs) const
    {
        return x <= rhs.x && y <= rhs.y && z <= rhs.z;
    }

    bool vec3::operator>=(const vec3& rhs) const
    {
        return x >= rhs.x && y >= rhs.y && z >= rhs.z;
    }

    vec3& vec3::operator=(const vec4& rhs)
    {
        this->x = rhs.x;
        this->y = rhs.y;
        this->z = rhs.z;
        return *this;
    }

    mat4::mat4(std::initializer_list<std::initializer_list<float>> rows)
    {
        int i = 0;
        for (const auto& row : rows)
        {
            int j = 0;
            for (float val : row)
                data[i * 4 + j++] = val;
            ++i;
        }
    }

    mat4& mat4::operator+=(const mat4& rhs)
    {
        for (int i = 0; i < 16; ++i)
            data[i] += rhs.data[i];
        return *this;
    }

    mat4& mat4::operator*=(const mat4& rhs)
    {
        mat4 result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
            {
                result.data[i * 4 + j] = 0;
                for (int k = 0; k < 4; ++k)
                    result.data[i * 4 + j] += data[i * 4 + k] * rhs.data[k * 4 + j];
            }
        *this = result;
        return *this;
    }

    mat4 mat4::operator*(const mat4& rhs) const
    {
        mat4 toReturn(*this);
        return toReturn *= rhs;
    }

    vec4 mat4::operator*(const vec4& v) const
    {
        float res_x = data[0]  * v.x + data[1]  * v.y + data[2]  * v.z + data[3]  * v.w;
        float res_y = data[4]  * v.x + data[5]  * v.y + data[6]  * v.z + data[7]  * v.w;
        float res_z = data[8]  * v.x + data[9]  * v.y + data[10] * v.z + data[11] * v.w;
        float res_w = data[12] * v.x + data[13] * v.y + data[14] * v.z + data[15] * v.w;
        return vec4{res_x, res_y, res_z, res_w};
    }

    vec4 vec4::operator*(const mat4& m) const
    {
        float res_x = m.data[0] * x + m.data[4] * y + m.data[8]  * z + m.data[12] * w;
        float res_y = m.data[1] * x + m.data[5] * y + m.data[9]  * z + m.data[13] * w;
        float res_z = m.data[2] * x + m.data[6] * y + m.data[10] * z + m.data[14] * w;
        float res_w = m.data[3] * x + m.data[7] * y + m.data[11] * z + m.data[15] * w;
        return vec4{res_x, res_y, res_z, res_w};
    }

    vec4 vec4::operator*=(const mat4& rhs)
    {
        *this = *this * rhs;
        return *this;
    }

    vec4 vec4::operator-(const vec4& rhs) const
    {
        return {x - rhs.x, y - rhs.y, z - rhs.z, w - rhs.w};
    }

    vec4 vec4::operator+(const vec4& rhs) const
    {
        return {x + rhs.x, y + rhs.y, z + rhs.z, w + rhs.w};
    }

    vec4& vec4::operator+=(const vec4& rhs)
    {
        x += rhs.x;
        y += rhs.y;
        z += rhs.z;
        w += rhs.w;
        return *this;
    }

    vec4& vec4::operator-=(const vec4& rhs)
    {
        x -= rhs.x;
        y -= rhs.y;
        z -= rhs.z;
        w -= rhs.w;
        return *this;
    }

    vec4 vec4::operator*(const float rhs) const
    {
        return {x * rhs, y * rhs, z * rhs, w * rhs};	
    }

    vec4 vec4::operator/(const float rhs) const
    {
        return {x / rhs, y / rhs, z / rhs, w / rhs};
    }


} // namespace slib