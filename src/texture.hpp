#pragma once
#include <algorithm>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>
#include "vendor/nothings/stb_image.h"

enum class TextureFilter {
    NEIGHBOUR,
    BILINEAR,
    BILINEAR_INT
};

struct RGBA8 {
    uint8_t r, g, b, a;
};

class Texture;
using SampleFn = void (Texture::*)(float, float, float&, float&, float&) const;

class Texture {
public:
    int w = 0;
    int h = 0;
    std::vector<unsigned char> data;
    SampleFn sampleFn = &Texture::sampleNearest; // Function pointer to avoid branch

    static Texture loadFromFile(const std::string& filename) {
        int width = 0;
        int height = 0;
        int channels = 0;
        unsigned char* imageData = stbi_load(filename.c_str(), &width, &height, &channels, 4);

        if (!imageData) {
            std::cout << "Failed to load image: " << filename
                      << " - " << stbi_failure_reason() << std::endl;
            return {};
        }

        std::vector<unsigned char> image(imageData, imageData + (width * height * 4));
        stbi_image_free(imageData);

        Texture texture;
        texture.w = width;
        texture.h = height;
        texture.data = std::move(image);
        return texture;
    }

    // Get pixels as RGBA8 array
    const RGBA8* pixels() const {
        return reinterpret_cast<const RGBA8*>(data.data());
    }

    // Wrap UV coordinate to [0, 1] range (for texture repeat)
    static float wrapUV(float uv) {
        uv = uv - static_cast<int>(uv); // fmod equivalent for positive
        if (uv < 0.0f) uv += 1.0f;      // handle negative UVs
        return uv;
    }

    // Check if texture is valid
    inline bool isValid() const { return w > 0 && h > 0 && !data.empty(); }

    // Sample texture at normalized coordinates (u, v) in [0, 1]
    // Returns RGB values in [0, 255] as floats
    void sampleNearest(float u, float v, float& r, float& g, float& b) const {
        if (!isValid()) {
            r = 255.0f; g = 0.0f; b = 255.0f; // Magenta for missing texture
            return;
        }

        u = wrapUV(u);
        v = wrapUV(v);

        int x = static_cast<int>(u * (w - 1));
        int y = static_cast<int>(v * (h - 1));

        const RGBA8& px = pixels()[y * w + x];

        r = static_cast<float>(px.r);
        g = static_cast<float>(px.g);
        b = static_cast<float>(px.b);
    }

    // Sample texture with bilinear filtering at normalized coordinates (u, v) in [0, 1]
    // Returns RGB values in [0, 255] as floats for interpolation precision
    void sampleBilinear(float u, float v, float& r, float& g, float& b) const {
        if (!isValid() || w < 2 || h < 2) {
            r = 255.0f; g = 0.0f; b = 255.0f; // Magenta for missing texture
            return;
        }

        u = wrapUV(u);
        v = wrapUV(v);

        // Map to texel space and center on texel centers (-0.5)
        float xf = u * w - 0.5f;
        float yf = v * h - 0.5f;

        // Clamp to keep (x+1, y+1) in-bounds
        int x = std::clamp(static_cast<int>(xf), 0, w - 2);
        int y = std::clamp(static_cast<int>(yf), 0, h - 2);

        float fx = xf - x;
        float fy = yf - y;

        // Direct 2D indexing with RGBA8 pointer
        const RGBA8* px = pixels();
        const RGBA8* rowT8 = px + y * w;
        const RGBA8* rowB8 = px + (y + 1) * w;

        // Load 4 neighboring pixels
        const RGBA8& p00 = rowT8[x];
        const RGBA8& p10 = rowT8[x + 1];
        const RGBA8& p01 = rowB8[x];
        const RGBA8& p11 = rowB8[x + 1];

        // Direct byte access - no bit shifting needed
        float r00 = static_cast<float>(p00.r);
        float g00 = static_cast<float>(p00.g);
        float b00 = static_cast<float>(p00.b);

        float r10 = static_cast<float>(p10.r);
        float g10 = static_cast<float>(p10.g);
        float b10 = static_cast<float>(p10.b);

        float r01 = static_cast<float>(p01.r);
        float g01 = static_cast<float>(p01.g);
        float b01 = static_cast<float>(p01.b);

        float r11 = static_cast<float>(p11.r);
        float g11 = static_cast<float>(p11.g);
        float b11 = static_cast<float>(p11.b);

        // Precompute interpolation weights
        float fx1 = 1.0f - fx;
        float fy1 = 1.0f - fy;

        // Bilinear interpolation using weighted sum (faster than nested lerps)
        r = (r00 * fx1 + r10 * fx) * fy1 + (r01 * fx1 + r11 * fx) * fy;
        g = (g00 * fx1 + g10 * fx) * fy1 + (g01 * fx1 + g11 * fx) * fy;
        b = (b00 * fx1 + b10 * fx) * fy1 + (b01 * fx1 + b11 * fx) * fy;
    }

    // Bilinear filtering using integer quadrant selection (https://github.com/tsoding/olive.c/blob/master/olive.c#L957)
    // Determines which quadrant of the texel center the sample falls in,
    // picks the 4 neighbors accordingly, and blends with integer weights
    // Useful for performance-critical sampling where floating-point interpolation is too costly
    void sampleBilinearInt(float u, float v, float& r, float& g, float& b) const {
        if (!isValid() || w < 2 || h < 2) {
            r = 255.0f; g = 0.0f; b = 255.0f;
            return;
        }

        u = wrapUV(u);
        v = wrapUV(v);

        // Sub-texel precision scale
        constexpr int S = 256;

        int nx = static_cast<int>(u * w * S);
        int ny = static_cast<int>(v * h * S);

        int px = nx % S;
        int py = ny % S;

        int x1 = nx / S, x2 = nx / S;
        int y1 = ny / S, y2 = ny / S;

        if (px < S / 2) {
            px += S / 2;
            x1 -= 1;
            if (x1 < 0) x1 = 0;
        } else {
            px -= S / 2;
            x2 += 1;
            if (x2 >= w) x2 = w - 1;
        }

        if (py < S / 2) {
            py += S / 2;
            y1 -= 1;
            if (y1 < 0) y1 = 0;
        } else {
            py -= S / 2;
            y2 += 1;
            if (y2 >= h) y2 = h - 1;
        }

        const RGBA8* px_data = pixels();
        const RGBA8& p00 = px_data[y1 * w + x1];
        const RGBA8& p10 = px_data[y1 * w + x2];
        const RGBA8& p01 = px_data[y2 * w + x1];
        const RGBA8& p11 = px_data[y2 * w + x2];

        // mix(a, b, t, range) = a + (b - a) * t / range
        auto mix = [S](float a, float b, int t) -> float {
            return a + (b - a) * t / S;
        };

        float topR = mix(p00.r, p10.r, px);
        float topG = mix(p00.g, p10.g, px);
        float topB = mix(p00.b, p10.b, px);

        float botR = mix(p01.r, p11.r, px);
        float botG = mix(p01.g, p11.g, px);
        float botB = mix(p01.b, p11.b, px);

        r = mix(topR, botR, py);
        g = mix(topG, botG, py);
        b = mix(topB, botB, py);
    }

    // Set filter mode (updates function pointer)
    void setFilter(TextureFilter filter) {
        switch (filter) {
            case TextureFilter::BILINEAR:     sampleFn = &Texture::sampleBilinear;    break;
            case TextureFilter::BILINEAR_INT: sampleFn = &Texture::sampleBilinearInt; break;
            default:                          sampleFn = &Texture::sampleNearest;     break;
        }
    }

    // Unified sample method - no branch, uses function pointer
    void sample(float u, float v, float& r, float& g, float& b) const {
        (this->*sampleFn)(u, v, r, g, b);
    }
};
