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
#include "pt/gpu_tracer.hpp"
#include "constants.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <string>
#include <vector>
#include <unistd.h>   // readlink

// Returns the directory containing the running executable
static std::string exeDir() {
    char buf[4096];
    ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) return ".";
    buf[n] = '\0';
    std::string p(buf);
    auto slash = p.rfind('/');
    return (slash == std::string::npos) ? "." : p.substr(0, slash);
}

// ---------------------------------------------------------------------------
// Scene directory scan
// ---------------------------------------------------------------------------

static std::vector<std::string> gScenePaths;
static std::vector<std::string> gSceneNames;

static void scanScenes(const std::string& dir) {
    namespace fs = std::filesystem;
    gScenePaths.clear(); gSceneNames.clear();
    if (!fs::exists(dir) || !fs::is_directory(dir)) return;
    std::vector<fs::directory_entry> entries;
    for (const auto& e : fs::directory_iterator(dir))
        if (e.is_regular_file()) {
            auto ext = e.path().extension().string();
            if (ext == ".yaml" || ext == ".yml") entries.push_back(e);
        }
    std::sort(entries.begin(), entries.end(),
              [](const fs::directory_entry& a, const fs::directory_entry& b){
                  return a.path().filename() < b.path().filename(); });
    for (const auto& e : entries) {
        gScenePaths.push_back(e.path().string());
        gSceneNames.push_back(e.path().stem().string());
    }
}

// ---------------------------------------------------------------------------
// App state shared with GLFW callbacks
// ---------------------------------------------------------------------------

struct AppState {
    PBRScene*      scene       = nullptr;
    GPUPathTracer* tracer      = nullptr;
    int*           sampleCount = nullptr;
    bool           dragging    = false;
    double         lastX       = 0.0, lastY = 0.0;
};

static void mouseButtonCb(GLFWwindow* w, int button, int action, int) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    auto* app = (AppState*)glfwGetWindowUserPointer(w);
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        app->dragging = (action == GLFW_PRESS);
        if (app->dragging) glfwGetCursorPos(w, &app->lastX, &app->lastY);
    }
}

static void cursorPosCb(GLFWwindow* w, double x, double y) {
    auto* app = (AppState*)glfwGetWindowUserPointer(w);
    if (!app->dragging || ImGui::GetIO().WantCaptureMouse) {
        app->lastX = x; app->lastY = y; return;
    }
    double dx = x - app->lastX, dy = y - app->lastY;
    app->lastX = x; app->lastY = y;
    app->scene->camera.orbitAzimuth  -= (float)dx * 0.005f;
    app->scene->camera.orbitElevation = std::clamp(
        app->scene->camera.orbitElevation + (float)dy * 0.005f, -1.5f, 1.5f);
    app->scene->camera.applyOrbit();
    app->tracer->reset(); *app->sampleCount = 0;
}

static void scrollCb(GLFWwindow* w, double, double dy) {
    if (ImGui::GetIO().WantCaptureMouse) return;
    auto* app = (AppState*)glfwGetWindowUserPointer(w);
    app->scene->camera.orbitRadius = std::max(0.1f,
        app->scene->camera.orbitRadius - (float)dy * 0.15f);
    app->scene->camera.applyOrbit();
    app->tracer->reset(); *app->sampleCount = 0;
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int, char**) {
    glfwSetErrorCallback([](int e, const char* d){
        std::fprintf(stderr, "GLFW Error %d: %s\n", e, d); });
    if (!glfwInit()) return 1;

    // OpenGL 4.3 core (required for compute shaders + SSBOs)
    const char* glsl_version = "#version 430";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    constexpr int W = SCREEN_WIDTH;
    constexpr int H = SCREEN_HEIGHT;

    float scale = 1.0f;
    if (GLFWmonitor* mon = glfwGetPrimaryMonitor())
        glfwGetMonitorContentScale(mon, &scale, nullptr);

    GLFWwindow* window = glfwCreateWindow(
        (int)(W * 2 * scale), (int)(H * 2 * scale),
        "PBR Path Tracer (GPU)", nullptr, nullptr);
    if (!window) { glfwTerminate(); return 1; }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // ------------------------------------------------------------------
    // Resource paths (relative to executable, not CWD)
    // ------------------------------------------------------------------
    const std::string resDir    = exeDir() + "/resources";
    const std::string scenesDir = resDir   + "/scenes";
    const std::string shadersDir= resDir   + "/shaders";

    // ------------------------------------------------------------------
    // Scene list
    // ------------------------------------------------------------------
    scanScenes(scenesDir);
    if (gScenePaths.empty()) {
        std::fprintf(stderr, "No YAML scenes found in '%s'\n", scenesDir.c_str());
        return 1;
    }

    int currentSceneIdx = 0;
    auto loadScene = [&](int idx) -> std::unique_ptr<PBRScene> {
        try { return PBRSceneLoader::loadFromFile(gScenePaths[idx]); }
        catch (const std::exception& ex) {
            std::fprintf(stderr, "Scene load error: %s\n", ex.what());
            return nullptr;
        }
    };

    auto scene = loadScene(currentSceneIdx);
    if (!scene) return 1;

    // ------------------------------------------------------------------
    // GPU path tracer
    // ------------------------------------------------------------------
    GPUPathTracer gpuTracer;
    int sampleCount = 0;

    auto resetAccum = [&]() { gpuTracer.reset(); sampleCount = 0; };

    // ------------------------------------------------------------------
    // GLFW callbacks — register BEFORE ImGui init so ImGui can chain them
    // ------------------------------------------------------------------
    AppState appState;
    appState.scene       = scene.get();
    appState.tracer      = &gpuTracer;
    appState.sampleCount = &sampleCount;
    glfwSetWindowUserPointer(window, &appState);
    glfwSetMouseButtonCallback(window, mouseButtonCb);
    glfwSetCursorPosCallback(window, cursorPosCb);
    glfwSetScrollCallback(window, scrollCb);

    // ------------------------------------------------------------------
    // Dear ImGui — init AFTER callbacks so ImGui chains them
    // ------------------------------------------------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // ------------------------------------------------------------------
    // Init GPU tracer + upload scene
    // ------------------------------------------------------------------
    if (!gpuTracer.init(W, H, shadersDir)) return 1;
    gpuTracer.uploadScene(*scene);

    // ------------------------------------------------------------------
    // Main loop
    // ------------------------------------------------------------------
    bool useNEE   = true;
    BRDFMode brdfMode = BRDFMode::GGX;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
            glfwSetWindowShouldClose(window, GLFW_TRUE);

        // ---- GPU: trace one sample per pixel ----
        sampleCount++;
        gpuTracer.dispatch(sampleCount, scene->camera, brdfMode, useNEE);

        // ---- ImGui frame ----
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---- Control panel ----
        ImGui::SetNextWindowBgAlpha(0.75f);
        ImGui::SetNextWindowPos({8.0f, 8.0f});
        ImGui::Begin("PBR Path Tracer", nullptr,
                     ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoMove);

        ImGui::Text("Scene: %s", scene->name.c_str());
        ImGui::Text("SPP:   %d", sampleCount);
        ImGui::Text("FPS:   %.1f", io.Framerate);
        ImGui::Text("Tris:  %zu   Spheres: %zu",
                    scene->triangles.size(), scene->spheres.size());
        ImGui::Separator();

        {
            auto getter = [](void* d, int i) -> const char* {
                return (*static_cast<std::vector<std::string>*>(d))[i].c_str(); };
            int prev = currentSceneIdx;
            ImGui::SetNextItemWidth(180.0f);
            if (ImGui::Combo("##scene", &currentSceneIdx, getter,
                             (void*)&gSceneNames, (int)gSceneNames.size())) {
                if (currentSceneIdx != prev) {
                    auto ns = loadScene(currentSceneIdx);
                    if (ns) {
                        scene = std::move(ns);
                        gpuTracer.uploadScene(*scene);
                        appState.scene = scene.get();
                        resetAccum();
                    } else currentSceneIdx = prev;
                }
            }
            ImGui::SameLine();
            if (ImGui::Button("Reset")) resetAccum();
        }

        ImGui::Separator();
        {
            int m = (brdfMode == BRDFMode::GGX) ? 1 : 0;
            ImGui::Text("BRDF:"); ImGui::SameLine();
            bool ch  = ImGui::RadioButton("Lambertian", &m, 0); ImGui::SameLine();
            ch      |= ImGui::RadioButton("GGX",        &m, 1);
            if (ch) { brdfMode = (m==1) ? BRDFMode::GGX : BRDFMode::Lambertian; resetAccum(); }
        }
        ImGui::Separator();
        { bool p=useNEE; ImGui::Checkbox("NEE (direct light)", &useNEE); if(useNEE!=p) resetAccum(); }
        ImGui::Separator();
        ImGui::TextDisabled("Drag: orbit   Scroll: zoom   Esc: quit");
        ImGui::End();

        // ---- Render ----
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClear(GL_COLOR_BUFFER_BIT);

        // Draw tone-mapped path-traced image
        gpuTracer.present(sampleCount);

        // Draw ImGui on top
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // ------------------------------------------------------------------
    // Cleanup
    // ------------------------------------------------------------------
    gpuTracer.destroy();
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
