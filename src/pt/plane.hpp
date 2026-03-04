#pragma once
#include "math.hpp"
#include <cmath>

// Infinite plane — intersected analytically (SDF converges in one step)
struct Plane {
    Vec3  normal{0.f, 1.f, 0.f};  // normalised; default Y-up (floor)
    float offset = 0.0f;           // dot(any_point_on_plane, normal)
    int   matId  = 0;
};
