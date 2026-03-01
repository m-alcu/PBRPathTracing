#include "scene_loader.hpp"
#include <yaml-cpp/yaml.h>
#include "vendor/tinyobjloader/tiny_obj_loader.h"
#include <filesystem>
#include <stdexcept>
#include <unordered_map>
#include <cstdio>

namespace PBRSceneLoader {

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static Vec3 parseVec3(const YAML::Node& n) {
    return {n[0].as<float>(), n[1].as<float>(), n[2].as<float>()};
}

static Material parseMaterial(const YAML::Node& node) {
    Material m;
    if (node["albedo"])   m.albedo   = parseVec3(node["albedo"]);
    if (node["emission"]) m.emission = parseVec3(node["emission"]);
    return m;
}

// Load an OBJ file via tinyobjloader, append triangles with the given matId.
// normals are generated per-face if absent.
static void loadObj(const std::string& filename, int matId,
                    std::vector<Triangle>& tris) {
    tinyobj::ObjReaderConfig cfg;
    cfg.triangulate = true;

    tinyobj::ObjReader reader;
    if (!reader.ParseFromFile(filename, cfg)) {
        std::fprintf(stderr, "[scene_loader] Failed to load '%s': %s\n",
                     filename.c_str(), reader.Error().c_str());
        return;
    }
    if (!reader.Warning().empty())
        std::fprintf(stderr, "[scene_loader] Warning loading '%s': %s\n",
                     filename.c_str(), reader.Warning().c_str());

    const auto& attrib = reader.GetAttrib();
    const auto& shapes = reader.GetShapes();

    for (const auto& shape : shapes) {
        size_t idxOff = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            int fv = (int)shape.mesh.num_face_vertices[f];
            if (fv != 3) { idxOff += fv; continue; }  // skip non-triangles

            Triangle tri;
            tri.matId = matId;
            bool hasNormals = true;

            for (int v = 0; v < 3; v++) {
                tinyobj::index_t idx = shape.mesh.indices[idxOff + v];
                tri.v[v] = {
                    attrib.vertices[3 * idx.vertex_index + 0],
                    attrib.vertices[3 * idx.vertex_index + 1],
                    attrib.vertices[3 * idx.vertex_index + 2]
                };
                if (idx.normal_index >= 0) {
                    tri.n[v] = {
                        attrib.normals[3 * idx.normal_index + 0],
                        attrib.normals[3 * idx.normal_index + 1],
                        attrib.normals[3 * idx.normal_index + 2]
                    };
                } else {
                    hasNormals = false;
                }
            }

            // Compute face normal if OBJ had none
            if (!hasNormals) {
                Vec3 e1 = tri.v[1] - tri.v[0];
                Vec3 e2 = tri.v[2] - tri.v[0];
                Vec3 n  = normalize(cross(e1, e2));
                tri.n[0] = tri.n[1] = tri.n[2] = n;
            }

            tris.push_back(tri);
            idxOff += 3;
        }
    }
}

// ---------------------------------------------------------------------------
// Main loader
// ---------------------------------------------------------------------------

std::unique_ptr<PBRScene> loadFromFile(const std::string& yamlPath) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(yamlPath);
    } catch (const YAML::Exception& e) {
        throw std::runtime_error("Failed to load '" + yamlPath + "': " + e.what());
    }

    if (!root["scene"])
        throw std::runtime_error("YAML missing top-level 'scene' key: " + yamlPath);

    YAML::Node sn = root["scene"];
    auto scene = std::make_unique<PBRScene>();

    if (sn["name"]) scene->name = sn["name"].as<std::string>();

    // ---- Materials -------------------------------------------------------
    std::unordered_map<std::string, int> matIndex;
    if (sn["materials"]) {
        for (const auto& mn : sn["materials"]) {
            int idx = (int)scene->materials.size();
            if (mn["name"]) matIndex[mn["name"].as<std::string>()] = idx;
            scene->materials.push_back(parseMaterial(mn));
        }
    }
    if (scene->materials.empty())
        scene->materials.push_back(Material{});  // default grey

    auto resolveMat = [&](const YAML::Node& node) -> int {
        if (!node["material"]) return 0;
        auto key = node["material"].as<std::string>();
        auto it  = matIndex.find(key);
        return (it != matIndex.end()) ? it->second : 0;
    };

    // ---- Camera ----------------------------------------------------------
    if (sn["camera"]) {
        auto cam = sn["camera"];
        Vec3 eye{0.0f, 0.8f, 4.0f}, target{0.0f, 0.0f, 0.0f};
        if (cam["position"]) eye    = parseVec3(cam["position"]);
        if (cam["target"])   target = parseVec3(cam["target"]);
        if (cam["fov"])      scene->camera.fov = cam["fov"].as<float>();
        scene->camera.setLookAt(eye, target);
    }

    // ---- Objects ---------------------------------------------------------
    if (sn["objects"]) {
        for (const auto& on : sn["objects"]) {
            std::string type = on["type"].as<std::string>();
            int matId = resolveMat(on);

            if (type == "obj") {
                std::string file = on["file"].as<std::string>();
                loadObj(file, matId, scene->triangles);

            } else if (type == "sphere") {
                Sphere sph;
                sph.matId  = matId;
                sph.radius = on["radius"] ? on["radius"].as<float>() : 1.0f;
                if (on["center"]) sph.center = parseVec3(on["center"]);
                scene->spheres.push_back(sph);

            } else {
                std::fprintf(stderr, "[scene_loader] Unknown object type '%s'\n",
                             type.c_str());
            }
        }
    }

    // Build BVH over triangle soup
    scene->buildBVH();

    std::fprintf(stdout, "[scene_loader] Loaded '%s': %zu triangles, %zu spheres, %zu materials\n",
                 scene->name.c_str(),
                 scene->triangles.size(),
                 scene->spheres.size(),
                 scene->materials.size());

    return scene;
}

} // namespace PBRSceneLoader
