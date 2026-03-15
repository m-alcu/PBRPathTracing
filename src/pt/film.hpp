#pragma once
#include "math.hpp"
#include "sampler.hpp"
#include <vector>
#include <cassert>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Pixel — accumulates radiance and sample count for one image pixel
// ---------------------------------------------------------------------------

struct Pixel {
    Vec3  sumRadiance{0.0f, 0.0f, 0.0f};
    int   sampleCount = 0;
};

// ---------------------------------------------------------------------------
// Film — CPU image buffer, PBRT-style
//
// addSample()     — splat one (CameraSample, radiance) pair
// getColor()      — return averaged, tone-mapped RGB in [0,1]
// reset()         — clear all pixels
// toRGB8()        — produce a flat RGBRGB... byte buffer (for display / export)
// ---------------------------------------------------------------------------

struct Film {
    int width, height;

    explicit Film(int w, int h) : width(w), height(h), pixels(w * h) {}

    // Record one radiance sample at the continuous film position given by s.
    // The sample lands in the pixel at floor(pFilmX), floor(pFilmY).
    void addSample(const CameraSample& s, const Vec3& radiance) {
        int px = (int)std::floor(s.pFilmX);
        int py = (int)std::floor(s.pFilmY);
        if (px < 0 || px >= width || py < 0 || py >= height) return;
        Pixel& p = pixels[py * width + px];
        p.sumRadiance += radiance;
        p.sampleCount++;
    }

    // Return averaged radiance for pixel (px, py). Returns black if no samples.
    Vec3 getRadiance(int px, int py) const {
        const Pixel& p = pixels[py * width + px];
        if (p.sampleCount == 0) return {0.0f, 0.0f, 0.0f};
        return p.sumRadiance / (float)p.sampleCount;
    }

    // Reinhard tone-map + gamma-correct one channel.
    static float tonemap(float v) {
        v = v / (1.0f + v);               // Reinhard
        return std::sqrt(std::clamp(v, 0.0f, 1.0f));  // gamma ~2.0
    }

    // Return tone-mapped colour in [0,1]^3.
    Vec3 getColor(int px, int py) const {
        Vec3 r = getRadiance(px, py);
        return { tonemap(r.x), tonemap(r.y), tonemap(r.z) };
    }

    // Clear all pixels — call when the camera or scene changes.
    void reset() {
        for (Pixel& p : pixels) {
            p.sumRadiance = {0.0f, 0.0f, 0.0f};
            p.sampleCount = 0;
        }
    }

    // Total samples accumulated across the film (all pixels).
    int totalSamples() const {
        int n = 0;
        for (const Pixel& p : pixels) n += p.sampleCount;
        return n;
    }

    // Samples per pixel for pixel (px,py).
    int spp(int px, int py) const {
        return pixels[py * width + px].sampleCount;
    }

    // Flatten to a byte buffer (R,G,B per pixel, row-major, top-left origin).
    // Useful for writing PNGs or uploading to an OpenGL texture.
    std::vector<uint8_t> toRGB8() const {
        std::vector<uint8_t> out(width * height * 3);
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) {
                Vec3 c = getColor(x, y);
                int base = (y * width + x) * 3;
                out[base + 0] = (uint8_t)(c.x * 255.0f + 0.5f);
                out[base + 1] = (uint8_t)(c.y * 255.0f + 0.5f);
                out[base + 2] = (uint8_t)(c.z * 255.0f + 0.5f);
            }
        return out;
    }

    // Flatten to BGRA uint32_t (matches GL_BGRA / GL_UNSIGNED_BYTE upload on
    // little-endian, preserving the same format as the old tonemapPixel path).
    std::vector<uint32_t> toBGRA32() const {
        std::vector<uint32_t> out(width * height);
        for (int y = 0; y < height; ++y)
            for (int x = 0; x < width; ++x) {
                Vec3 c = getColor(x, y);
                uint8_t r = (uint8_t)(c.x * 255.0f + 0.5f);
                uint8_t g = (uint8_t)(c.y * 255.0f + 0.5f);
                uint8_t b = (uint8_t)(c.z * 255.0f + 0.5f);
                // Memory layout: [B][G][R][A] = BGRA, as GL_BGRA expects.
                out[y * width + x] = (0xFFu << 24) | ((uint32_t)r << 16)
                                   | ((uint32_t)g << 8) | b;
            }
        return out;
    }

private:
    std::vector<Pixel> pixels;
};
