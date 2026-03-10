#pragma once
#include "math.hpp"

struct Material {
    Vec3  albedo{0.8f, 0.8f, 0.8f};   // [0..1] diffuse reflectance / metal tint
    Vec3  emission{0.0f, 0.0f, 0.0f}; // emitted radiance (area lights)
    float metallic{0.0f};              // 0 = pure diffuse, 1 = pure specular
    float roughness{0.0f};             // 0 = perfect mirror, 1 = fully rough metal
    float ior{1.5f};                   // index of refraction (glass ≈ 1.5)
    float transmission{0.0f};          // 0 = opaque, 1 = fully transmissive (dielectric)
    int   albedoTex{-1};               // index into PBRScene::textures, -1 = none

    // Analytical anti-aliased checkerboard (checkersGradBox)
    bool  checker{false};
    float checkerScale{4.0f};              // checker tiles per world unit
    Vec3  checkerAlbedo2{0.1f, 0.1f, 0.1f}; // second checker color
};
