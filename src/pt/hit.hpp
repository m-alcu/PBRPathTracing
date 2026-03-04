#pragma once
#include "math.hpp"

struct Hit {
    float t     = 1e30f;
    Vec3  p{}, n{};
    float tu = 0.0f, tv = 0.0f;  // interpolated texture UV at hit point
    int   matId = 0;
    bool  hit   = false;
};
