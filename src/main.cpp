#include <SDL3/SDL.h>

#include "vendor/imgui/imgui.h"
#include "vendor/imgui/imgui_impl_sdl3.h"
#include "vendor/imgui/imgui_impl_sdlrenderer3.h"

#include "pt/scene_loader.hpp"
#include "pt/tracer.hpp"
#include "pt/rng.hpp"
#include "constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <thread>
#include <vector>

// ---------------------------------------------------------------------------
// Scene directory scan
// ---------------------------------------------------------------------------

static std::vector<std::string> gScenePaths;
static std::vector<std::string> gSceneNames;

static void scanScenes(const std::string& dir) {
    namespace fs = std::filesystem;
    gScenePaths.clear();
    gSceneNames.clear();
    if (!fs::exists(dir) || !fs::is_directory(dir)) return;

    std::vector<fs::directory_entry> entries;
    for (const auto& e : fs::directory_iterator(dir))
        if (e.is_regular_file()) {
            auto ext = e.path().extension().string();
            if (ext == ".yaml" || ext == ".yml")
                entries.push_back(e);
        }
    std::sort(entries.begin(), entries.end(),
              [](const fs::directory_entry& a, const fs::directory_entry& b) {
                  return a.path().filename() < b.path().filename();
              });
    for (const auto& e : entries) {
        gScenePaths.push_back(e.path().string());
        gSceneNames.push_back(e.path().stem().string());
    }
}

// ---------------------------------------------------------------------------
// Tone mapping
// ---------------------------------------------------------------------------

// Reinhard + gamma-2 (sqrt) tone mapping
inline uint32_t tonemapPixel(Vec3 c, float invSpp) {
    c *= invSpp;
    // Reinhard per-channel
    c.x = c.x / (1.0f + c.x);
    c.y = c.y / (1.0f + c.y);
    c.z = c.z / (1.0f + c.z);
    // Gamma 2 (approximate sRGB)
    auto g = [](float x) { return std::sqrt(std::clamp(x, 0.0f, 1.0f)); };
    uint8_t r = (uint8_t)(g(c.x) * 255.0f + 0.5f);
    uint8_t gn = (uint8_t)(g(c.y) * 255.0f + 0.5f);
    uint8_t b  = (uint8_t)(g(c.z) * 255.0f + 0.5f);
    return (0xFFu << 24) | ((uint32_t)r << 16) | ((uint32_t)gn << 8) | b;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int, char**) {
    // ------------------------------------------------------------------
    // SDL3 init
    // ------------------------------------------------------------------
    if (!SDL_Init(SDL_INIT_VIDEO)) {
        std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    constexpr int W = SCREEN_WIDTH;
    constexpr int H = SCREEN_HEIGHT;

    SDL_Window* window = SDL_CreateWindow(
        "PBR Path Tracer",
        W * 2, H * 2,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_HIDDEN | SDL_WINDOW_HIGH_PIXEL_DENSITY);
    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, nullptr);
    if (!renderer) {
        std::fprintf(stderr, "SDL_CreateRenderer: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetRenderVSync(renderer, 1);

    // Streaming texture that holds the rendered image
    SDL_Texture* texture = SDL_CreateTexture(
        renderer, SDL_PIXELFORMAT_ARGB8888,
        SDL_TEXTUREACCESS_STREAMING, W, H);
    if (!texture) {
        std::fprintf(stderr, "SDL_CreateTexture: %s\n", SDL_GetError());
        return 1;
    }
    SDL_SetTextureBlendMode(texture, SDL_BLENDMODE_NONE);
    SDL_SetWindowPosition(window, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED);
    SDL_ShowWindow(window);

    // ------------------------------------------------------------------
    // Dear ImGui
    // ------------------------------------------------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer3_Init(renderer);

    // ------------------------------------------------------------------
    // Scene list
    // ------------------------------------------------------------------
    scanScenes(SCENES_PATH);
    if (gScenePaths.empty()) {
        std::fprintf(stderr, "No YAML scenes found in '%s'\n", SCENES_PATH);
        return 1;
    }

    int  currentSceneIdx = 0;
    auto loadScene = [&](int idx) -> std::unique_ptr<PBRScene> {
        try {
            return PBRSceneLoader::loadFromFile(gScenePaths[idx]);
        } catch (const std::exception& ex) {
            std::fprintf(stderr, "Scene load error: %s\n", ex.what());
            return nullptr;
        }
    };

    auto scene = loadScene(currentSceneIdx);
    if (!scene) return 1;

    // ------------------------------------------------------------------
    // Accumulation buffer
    // ------------------------------------------------------------------
    std::vector<Vec3>     accum(W * H, Vec3{});
    std::vector<uint32_t> pixels(W * H, 0xFF000000u);
    int sampleCount = 0;

    auto resetAccum = [&]() {
        std::fill(accum.begin(), accum.end(), Vec3{});
        sampleCount = 0;
    };

    // ------------------------------------------------------------------
    // Threading
    // ------------------------------------------------------------------
    int nThreads = (int)std::thread::hardware_concurrency();
    if (nThreads < 1) nThreads = 1;

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    bool running    = true;
    bool useHashRng = true;   // false → PCG32
    BRDFMode brdfMode = BRDFMode::GGX;

    while (running) {
        // ---- Events --------------------------------------------------
        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            ImGui_ImplSDL3_ProcessEvent(&e);

            if (e.type == SDL_EVENT_QUIT) running = false;
            if (e.type == SDL_EVENT_KEY_DOWN &&
                e.key.key == SDLK_ESCAPE) running = false;

            if (!io.WantCaptureMouse) {
                // Left-drag: orbit
                if (e.type == SDL_EVENT_MOUSE_MOTION &&
                    (e.motion.state & SDL_BUTTON_LMASK)) {
                    scene->camera.orbitAzimuth   -= e.motion.xrel * 0.005f;
                    scene->camera.orbitElevation  = std::clamp(
                        scene->camera.orbitElevation + e.motion.yrel * 0.005f,
                        -1.5f, 1.5f);
                    scene->camera.applyOrbit();
                    resetAccum();
                }
                // Scroll: zoom
                if (e.type == SDL_EVENT_MOUSE_WHEEL) {
                    scene->camera.orbitRadius = std::max(0.1f,
                        scene->camera.orbitRadius - e.wheel.y * 0.15f);
                    scene->camera.applyOrbit();
                    resetAccum();
                }
            }
        }

        // ---- Render one pass (1 SPP, multi-threaded row-striping) ------
        sampleCount++;
        {
            const int rowsPerThread = (H + nThreads - 1) / nThreads;
            std::vector<std::thread> threads;
            threads.reserve(nThreads);

            for (int tid = 0; tid < nThreads; tid++) {
                int rowStart = tid * rowsPerThread;
                int rowEnd   = std::min(rowStart + rowsPerThread, H);
                threads.emplace_back([&, rowStart, rowEnd, tid]() {
                    for (int py = rowStart; py < rowEnd; py++) {
                        for (int px = 0; px < W; px++) {
                            // Unique seed per pixel × sample
                            if (useHashRng) {
                                uint32_t seed =
                                    ((uint32_t)py * (uint32_t)W + (uint32_t)px)
                                    ^ ((uint32_t)sampleCount * 2246822519u);
                                HashRNG rng(seed);
                                Ray ray = scene->camera.generateRay(
                                    px, py, W, H, rng.nextFloat(), rng.nextFloat());
                                accum[py * W + px] += tracePath(ray, rng, scene->materials, *scene, brdfMode);
                            } else {
                                uint64_t seed =
                                (uint64_t)((py * W + px) * 1099511628211ull
                                ^ (uint64_t)sampleCount * 6364136223846793005ull);                                     
                                RNG rng(seed);
                                Ray ray = scene->camera.generateRay(
                                    px, py, W, H, rng.nextFloat(), rng.nextFloat());
                                accum[py * W + px] += tracePath(ray, rng, scene->materials, *scene, brdfMode);
                            }
                        }
                    }
                });
            }
            for (auto& t : threads) t.join();
        }

        // ---- Tone map + upload texture --------------------------------
        float invN = 1.0f / (float)sampleCount;
        for (int i = 0; i < W * H; i++)
            pixels[i] = tonemapPixel(accum[i], invN);
        SDL_UpdateTexture(texture, nullptr, pixels.data(), 4 * W);

        // ---- ImGui ---------------------------------------------------
        ImGui_ImplSDLRenderer3_NewFrame();
        ImGui_ImplSDL3_NewFrame();
        ImGui::NewFrame();

        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::SetNextWindowPos({8.0f, 8.0f});
        ImGui::Begin("PBR Path Tracer", nullptr,
                     ImGuiWindowFlags_AlwaysAutoResize |
                     ImGuiWindowFlags_NoMove);

        ImGui::Text("Scene: %s", scene->name.c_str());
        ImGui::Text("SPP:   %d", sampleCount);
        ImGui::Text("FPS:   %.1f", io.Framerate);
        ImGui::Text("Threads: %d", nThreads);
        ImGui::Text("Tris:    %zu   Spheres: %zu",
                    scene->triangles.size(), scene->spheres.size());
        ImGui::Separator();

        // Scene combo
        {
            auto getter = [](void* data, int idx) -> const char* {
                return (*static_cast<std::vector<std::string>*>(data))[idx].c_str();
            };
            int prev = currentSceneIdx;
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::Combo("##scene", &currentSceneIdx, getter,
                             (void*)&gSceneNames, (int)gSceneNames.size())) {
                if (currentSceneIdx != prev) {
                    auto ns = loadScene(currentSceneIdx);
                    if (ns) { scene = std::move(ns); resetAccum(); }
                    else currentSceneIdx = prev;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset")) resetAccum();
        }

        ImGui::Separator();
        {
            int rngChoice = useHashRng ? 1 : 0;
            ImGui::Text("RNG:"); ImGui::SameLine();
            bool changed  = ImGui::RadioButton("PCG32", &rngChoice, 0); ImGui::SameLine();
            changed      |= ImGui::RadioButton("Hash",  &rngChoice, 1);
            if (changed) { useHashRng = (rngChoice == 1); resetAccum(); }
        }

        ImGui::Separator();
        {
            int modeChoice = (brdfMode == BRDFMode::GGX) ? 1 : 0;
            ImGui::Text("BRDF:"); ImGui::SameLine();
            bool changed  = ImGui::RadioButton("Lambertian", &modeChoice, 0); ImGui::SameLine();
            changed      |= ImGui::RadioButton("GGX",        &modeChoice, 1);
            if (changed) { brdfMode = (modeChoice == 1) ? BRDFMode::GGX : BRDFMode::Lambertian; resetAccum(); }
        }

        ImGui::Separator();
        ImGui::TextDisabled("Drag: orbit   Scroll: zoom   Esc: quit");
        ImGui::End();

        // ---- Present -------------------------------------------------
        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);
        SDL_RenderTexture(renderer, texture, nullptr, nullptr);
        ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    // ------------------------------------------------------------------
    // Cleanup
    // ------------------------------------------------------------------
    ImGui_ImplSDLRenderer3_Shutdown();
    ImGui_ImplSDL3_Shutdown();
    ImGui::DestroyContext();
    SDL_DestroyTexture(texture);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
