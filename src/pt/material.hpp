#pragma once
#include "math.hpp"

struct Material {
    Vec3  albedo{0.8f, 0.8f, 0.8f};   // [0..1] diffuse reflectance / metal tint
    Vec3  emission{0.0f, 0.0f, 0.0f}; // emitted radiance (area lights)
    float metallic{0.0f};              // 0 = pure diffuse, 1 = pure specular
    float roughness{0.0f};             // 0 = perfect mirror, 1 = fully rough metal
    int   albedoTex{-1};               // index into PBRScene::textures, -1 = none
};

struct Hit {
    float t     = 1e30f;
    Vec3  p{}, n{};
    float tu = 0.0f, tv = 0.0f;  // interpolated texture UV at hit point
    int   matId = 0;
    bool  hit   = false;
};
