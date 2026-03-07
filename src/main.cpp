#ifndef GL_GLEXT_PROTOTYPES
#define GL_GLEXT_PROTOTYPES
#endif
#include <GL/gl.h>
#include <GL/glext.h>
#include <GLFW/glfw3.h>

#include "vendor/imgui/imgui.h"
#include "vendor/imgui/imgui_impl_glfw.h"
#include "vendor/imgui/imgui_impl_opengl3.h"

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

// Reinhard + gamma-2 (sqrt) tone mapping — outputs BGRA bytes as uint32_t
inline uint32_t tonemapPixel(Vec3 c, float invSpp) {
    c *= invSpp;
    c.x = c.x / (1.0f + c.x);
    c.y = c.y / (1.0f + c.y);
    c.z = c.z / (1.0f + c.z);
    auto g = [](float x) { return std::sqrt(std::clamp(x, 0.0f, 1.0f)); };
    uint8_t r = (uint8_t)(g(c.x) * 255.0f + 0.5f);
    uint8_t gn = (uint8_t)(g(c.y) * 255.0f + 0.5f);
    uint8_t b  = (uint8_t)(g(c.z) * 255.0f + 0.5f);
    // BGRA layout (matches GL_BGRA GL_UNSIGNED_BYTE on little-endian)
    return (0xFFu << 24) | ((uint32_t)r << 16) | ((uint32_t)gn << 8) | b;
}

// ---------------------------------------------------------------------------
// App state shared with GLFW callbacks
// ---------------------------------------------------------------------------

struct AppState {
    PBRScene*          scene       = nullptr;
    std::vector<Vec3>* accum       = nullptr;
    int*               sampleCount = nullptr;
    bool               dragging    = false;
    double             lastX       = 0.0;
    double             lastY       = 0.0;
};

static void mouseButtonCb(GLFWwindow* w, int button, int action, int) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    auto* app = (AppState*)glfwGetWindowUserPointer(w);
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        app->dragging = (action == GLFW_PRESS);
        if (app->dragging)
            glfwGetCursorPos(w, &app->lastX, &app->lastY);
    }
}

static void cursorPosCb(GLFWwindow* w, double x, double y) {
    auto* app = (AppState*)glfwGetWindowUserPointer(w);
    if (!app->dragging || ImGui::GetIO().WantCaptureMouse) {
        app->lastX = x; app->lastY = y; return;
    }
    double dx = x - app->lastX;
    double dy = y - app->lastY;
    app->lastX = x; app->lastY = y;

    app->scene->camera.orbitAzimuth  -= (float)dx * 0.005f;
    app->scene->camera.orbitElevation = std::clamp(
        app->scene->camera.orbitElevation + (float)dy * 0.005f, -1.5f, 1.5f);
    app->scene->camera.applyOrbit();
    std::fill(app->accum->begin(), app->accum->end(), Vec3{});
    *app->sampleCount = 0;
}

static void scrollCb(GLFWwindow* w, double /*dx*/, double dy) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    auto* app = (AppState*)glfwGetWindowUserPointer(w);
    app->scene->camera.orbitRadius = std::max(0.1f,
        app->scene->camera.orbitRadius - (float)dy * 0.15f);
    app->scene->camera.applyOrbit();
    std::fill(app->accum->begin(), app->accum->end(), Vec3{});
    *app->sampleCount = 0;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int, char**) {
    glfwSetErrorCallback([](int err, const char* desc) {
        std::fprintf(stderr, "GLFW Error %d: %s\n", err, desc);
    });
    if (!glfwInit()) return 1;

    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

    constexpr int W = SCREEN_WIDTH;
    constexpr int H = SCREEN_HEIGHT;

    // HiDPI: scale window by monitor content scale
    float scale = 1.0f;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor())
        glfwGetMonitorContentScale(mon, &scale, nullptr);

    GLFWwindow* window = glfwCreateWindow(
        (int)(W * 2 * scale), (int)(H * 2 * scale),
        "PBR Path Tracer", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // vsync

    // ------------------------------------------------------------------
    // Dear ImGui
    // ------------------------------------------------------------------
    // ------------------------------------------------------------------
    // Scene list
    // ------------------------------------------------------------------
    scanScenes(SCENES_PATH);
    if (gScenePaths.empty()) {
        std::fprintf(stderr, "No YAML scenes found in '%s'\n", SCENES_PATH);
        return 1;
    }

    int currentSceneIdx = 0;
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
    // GLFW callbacks (orbit / zoom) — must be registered BEFORE ImGui
    // init so ImGui can chain to them via its PrevUserCallback mechanism.
    // ------------------------------------------------------------------
    AppState appState;
    appState.scene       = scene.get();
    appState.accum       = &accum;
    appState.sampleCount = &sampleCount;
    glfwSetWindowUserPointer(window, &appState);
    glfwSetMouseButtonCallback(window, mouseButtonCb);
    glfwSetCursorPosCallback(window, cursorPosCb);
    glfwSetScrollCallback(window, scrollCb);

    // ------------------------------------------------------------------
    // Dear ImGui — init AFTER registering our callbacks so it chains them
    // ------------------------------------------------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // ------------------------------------------------------------------
    // OpenGL texture for the path-traced image
    // ------------------------------------------------------------------
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    // GL_BGRA matches the ARGB8888 layout produced by tonemapPixel on little-endian
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, W, H, 0, GL_BGRA, GL_UNSIGNED_BYTE, nullptr);
    glBindTexture(GL_TEXTURE_2D, 0);

    // ------------------------------------------------------------------
    // Threading
    // ------------------------------------------------------------------
    int nThreads = (int)std::thread::hardware_concurrency();
    if (nThreads < 1) nThreads = 1;

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    bool useHashRng = true;
    BRDFMode brdfMode = BRDFMode::GGX;
    bool useNEE = true;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        // ---- Render one pass (1 SPP, multi-threaded row-striping) ------
        sampleCount++;
        {
            const int rowsPerThread = (H + nThreads - 1) / nThreads;
            std::vector<std::thread> threads;
            threads.reserve(nThreads);

            for (int tid = 0; tid < nThreads; tid++) {
                int rowStart = tid * rowsPerThread;
                int rowEnd   = std::min(rowStart + rowsPerThread, H);
                threads.emplace_back([&, rowStart, rowEnd]() {
                    for (int py = rowStart; py < rowEnd; py++) {
                        for (int px = 0; px < W; px++) {
                            if (useHashRng) {
                                uint32_t seed =
                                    ((uint32_t)py * (uint32_t)W + (uint32_t)px)
                                    ^ ((uint32_t)sampleCount * 2246822519u);
                                HashRNG rng(seed);
                                Ray ray = scene->camera.generateRay(
                                    px, py, W, H, rng.nextFloat(), rng.nextFloat());
                                accum[py * W + px] += tracePath(ray, rng, scene->materials, *scene, brdfMode, useNEE);
                            } else {
                                uint64_t seed =
                                    (uint64_t)((py * W + px) * 1099511628211ull
                                    ^ (uint64_t)sampleCount * 6364136223846793005ull);
                                RNG rng(seed);
                                Ray ray = scene->camera.generateRay(
                                    px, py, W, H, rng.nextFloat(), rng.nextFloat());
                                accum[py * W + px] += tracePath(ray, rng, scene->materials, *scene, brdfMode, useNEE);
                            }
                        }
                    }
                });
            }
            for (auto& t : threads) t.join();
        }

        // ---- Tone map + upload to GL texture --------------------------
        float invN = 1.0f / (float)sampleCount;
        for (int i = 0; i < W * H; i++)
            pixels[i] = tonemapPixel(accum[i], invN);

        glBindTexture(GL_TEXTURE_2D, tex);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, W, H, GL_BGRA, GL_UNSIGNED_BYTE, pixels.data());
        glBindTexture(GL_TEXTURE_2D, 0);

        // ---- ImGui frame ---------------------------------------------
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Draw path-traced image as fullscreen background
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        ImGui::GetBackgroundDrawList()->AddImage(
            (ImTextureID)(uintptr_t)tex,
            ImVec2(0.0f, 0.0f),
            ImVec2((float)display_w, (float)display_h));

        // Control panel
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
                    if (ns) {
                        scene = std::move(ns);
                        appState.scene = scene.get();
                        resetAccum();
                    } else {
                        currentSceneIdx = prev;
                    }
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
        {
            bool prev = useNEE;
            ImGui::Checkbox("NEE (direct light sampling)", &useNEE);
            if (useNEE != prev) resetAccum();
        }

        ImGui::Separator();
        ImGui::TextDisabled("Drag: orbit   Scroll: zoom   Esc: quit");
        ImGui::End();

        // ---- Present -------------------------------------------------
        ImGui::Render();
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ------------------------------------------------------------------
    // Cleanup
    // ------------------------------------------------------------------
    glDeleteTextures(1, &tex);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
