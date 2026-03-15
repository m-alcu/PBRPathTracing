#pragma once
#include "film.hpp"
#include "sampler.hpp"
#include "camera.hpp"
#include "scene.hpp"
#include "tracer.hpp"   // tracePath, renderDirect, BRDFMode

#include <algorithm>
#include <memory>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// SamplerIntegrator — PBRT-style base
//
// Subclasses implement Li() (incident radiance for one ray).
// render() drives the pixel loop: Sampler → Camera → Li() → Film.
// ---------------------------------------------------------------------------

struct SamplerIntegrator {
    virtual ~SamplerIntegrator() = default;

    // Compute incident radiance along `ray`. `sampler` supplies all random values.
    virtual Vec3 Li(const Ray& ray, Sampler& sampler, const PBRScene& scene) = 0;

    // Render one full pass (1 sample per pixel) into `film`, multi-threaded.
    // `prototype` is cloned once per thread; `passSeed` varies each pass so
    // threads don't repeat the same random streams across frames.
    void render(Film& film, const Camera& camera, const PBRScene& scene,
                Sampler& prototype, uint64_t passSeed, int nThreads) {
        const int W = film.width, H = film.height;
        const int rowsPerThread = (H + nThreads - 1) / nThreads;

        std::vector<std::thread> threads;
        threads.reserve(nThreads);

        for (int tid = 0; tid < nThreads; ++tid) {
            const int rowStart = tid * rowsPerThread;
            const int rowEnd   = std::min(rowStart + rowsPerThread, H);

            // Each thread gets an independent sampler clone.
            uint64_t threadSeed = passSeed ^ ((uint64_t)(tid + 1) * 6364136223846793005ull);
            auto sampler = prototype.clone(threadSeed);

            threads.emplace_back(
                [this, &film, &camera, &scene, rowStart, rowEnd, W, H,
                 s = std::move(sampler)]() mutable {
                    for (int py = rowStart; py < rowEnd; ++py) {
                        for (int px = 0; px < W; ++px) {
                            CameraSample cs = s->getCameraSample(px, py);
                            Ray ray = camera.generateRayFromSample(cs, W, H);
                            Vec3 L  = Li(ray, *s, scene);
                            film.addSample(cs, L);
                        }
                    }
                });
        }
        for (auto& t : threads) t.join();
    }
};

// ---------------------------------------------------------------------------
// PathIntegrator — full GGX path tracing (NEE + MIS, glass, BVH)
// ---------------------------------------------------------------------------

struct PathIntegrator : SamplerIntegrator {
    BRDFMode brdfMode       = BRDFMode::GGX;
    bool     useNEE         = true;
    float    pixelConeAngle = 0.002f;

    Vec3 Li(const Ray& ray, Sampler& sampler, const PBRScene& scene) override {
        // tracePath is templated on RNG; Sampler exposes nextFloat() so it works.
        return tracePath(ray, sampler, scene.materials, scene,
                         brdfMode, useNEE, pixelConeAngle);
    }
};

// ---------------------------------------------------------------------------
// DirectIntegrator — analytical direct shading (IQ-style, no Monte Carlo)
// ---------------------------------------------------------------------------

struct DirectIntegrator : SamplerIntegrator {
    float pixelConeAngle = 0.002f;
    bool  useSun         = true;
    bool  useSky         = true;
    bool  useBackFill    = true;
    bool  useRim         = true;
    Vec3  sunDir         = normalize(Vec3{-0.5f, 0.4f, -0.6f});

    Vec3 Li(const Ray& ray, Sampler& /*sampler*/, const PBRScene& scene) override {
        return renderDirect(ray, scene, scene.materials, pixelConeAngle,
                            useSun, useSky, useBackFill, useRim, sunDir);
    }
};
