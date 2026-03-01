#pragma once
#include "math.hpp"

struct Material {
    Vec3 albedo{0.8f, 0.8f, 0.8f};   // [0..1] diffuse reflectance
    Vec3 emission{0.0f, 0.0f, 0.0f}; // emitted radiance (area lights)
};

struct Hit {
    float t     = 1e30f;
    Vec3  p{}, n{};
    int   matId = 0;
    bool  hit   = false;
};
