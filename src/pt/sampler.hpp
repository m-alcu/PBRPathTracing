#pragma once
#include "rng.hpp"
#include <cstdint>
#include <memory>

// ---------------------------------------------------------------------------
// CameraSample — pixel position (subpixel) + lens position for DoF
// ---------------------------------------------------------------------------

struct CameraSample {
    float pFilmX, pFilmY;  // continuous position in pixel space, e.g. px + [0,1)
    float pLensU, pLensV;  // [0,1) for depth-of-field (unused — keep 0 for now)
};

// ---------------------------------------------------------------------------
// Sampler base interface
// ---------------------------------------------------------------------------

struct Sampler {
    virtual ~Sampler() = default;

    virtual float        get1D()                                   = 0;
    virtual CameraSample getCameraSample(int px, int py)           = 0;
    virtual std::unique_ptr<Sampler> clone(uint64_t seed) const    = 0;

    // Convenience alias so tracePath<Sampler> duck-typing works unchanged.
    float nextFloat() { return get1D(); }
};

// ---------------------------------------------------------------------------
// IndependentSampler — wraps PCG32 RNG; each sample is independent
// ---------------------------------------------------------------------------

struct IndependentSampler : Sampler {
    RNG rng;

    explicit IndependentSampler(uint64_t seed = 0xcafef00dd15ea5e5ull) : rng(seed) {}

    float get1D() override {
        return rng.nextFloat();
    }

    CameraSample getCameraSample(int px, int py) override {
        return {
            (float)px + rng.nextFloat(),
            (float)py + rng.nextFloat(),
            0.0f, 0.0f
        };
    }

    std::unique_ptr<Sampler> clone(uint64_t seed) const override {
        // Mix seed so small sequential values (tid=0,1,2…) produce independent streams.
        seed ^= seed >> 33; seed *= 0xff51afd7ed558ccdull; seed ^= seed >> 33;
        return std::make_unique<IndependentSampler>(seed);
    }
};

// ---------------------------------------------------------------------------
// HashSampler — re-seeds per pixel (stateless, reproducible, matches original
//               HashRNG behaviour from main.cpp).  Good for debugging / AOV.
// ---------------------------------------------------------------------------

struct HashSampler : Sampler {
    int      W;
    uint32_t samplePass;  // incremented each render pass by the caller
    HashRNG  rng{0};

    HashSampler(int w, uint32_t pass) : W(w), samplePass(pass) {}

    float get1D() override { return rng.nextFloat(); }

    // Re-seeds from pixel coords + pass so every pixel gets an independent stream.
    CameraSample getCameraSample(int px, int py) override {
        uint32_t seed = ((uint32_t)py * (uint32_t)W + (uint32_t)px)
                      ^ (samplePass * 2246822519u);
        rng = HashRNG(seed);
        return { (float)px + rng.nextFloat(), (float)py + rng.nextFloat(), 0.0f, 0.0f };
    }

    // clone() passes samplePass through the seed's lower 32 bits.
    std::unique_ptr<Sampler> clone(uint64_t seed) const override {
        return std::make_unique<HashSampler>(W, (uint32_t)seed);
    }
};
