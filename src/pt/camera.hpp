#pragma once
#include "math.hpp"
#include "constants.hpp"
#include <cmath>

struct Camera {
    Vec3  orbitTarget{0.0f, 0.0f, 0.0f};
    float orbitRadius    = 4.0f;
    float orbitAzimuth   = 0.0f;
    float orbitElevation = 0.2f;
    float fov = 45.0f;   // vertical FOV in degrees

    // Derived state – updated by applyOrbit() / setLookAt()
    Vec3 pos{0.0f, 0.8f, 4.0f};
    Vec3 forward{0.0f, 0.0f, -1.0f};

    void applyOrbit() {
        float ca = std::cos(orbitAzimuth),  sa = std::sin(orbitAzimuth);
        float ce = std::cos(orbitElevation), se = std::sin(orbitElevation);
        pos     = orbitTarget + Vec3{sa * ce, se, ca * ce} * orbitRadius;
        forward = normalize(orbitTarget - pos);
    }

    void setLookAt(Vec3 eye, Vec3 target) {
        orbitTarget = target;
        Vec3 d = eye - target;
        orbitRadius    = length(d);
        orbitAzimuth   = std::atan2(d.x, d.z);
        float safe = std::clamp(d.y / orbitRadius, -0.999f, 0.999f);
        orbitElevation = std::asin(safe);
        pos     = eye;
        forward = normalize(target - eye);
    }

    Ray generateRay(int px, int py, int w, int h, float du, float dv) const {
        Vec3 right = normalize(cross(forward, Vec3{0.0f, 1.0f, 0.0f}));
        if (length(right) < 0.001f) right = {1.0f, 0.0f, 0.0f};
        Vec3 up = cross(right, forward);

        float aspect = (float)w / (float)h;
        float scale  = std::tan(fov * 0.5f * RAD);
        float ndcX   = (2.0f * ((float)px + du) / (float)w - 1.0f) * aspect * scale;
        float ndcY   = (1.0f - 2.0f * ((float)py + dv) / (float)h) * scale;

        Vec3 dir = normalize(forward + right * ndcX + up * ndcY);
        return {pos, dir};
    }
};
