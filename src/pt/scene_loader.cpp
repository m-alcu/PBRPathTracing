#include "scene_loader.hpp"
#include <yaml-cpp/yaml.h>
#include "vendor/tinyobjloader/tiny_obj_loader.h"
#include "../texture.hpp"
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
    if (node["albedo"])    m.albedo    = parseVec3(node["albedo"]);
    if (node["emission"])  m.emission  = parseVec3(node["emission"]);
    if (node["metallic"])  m.metallic  = node["metallic"].as<float>();
    if (node["roughness"]) m.roughness = node["roughness"].as<float>();
    return m;
}

// Load an OBJ file via tinyobjloader, append triangles.
// When useMtl=true, materials from the accompanying .mtl are appended to
// outMaterials (and diffuse textures into outTextures via texCache).
// When useMtl=false, all triangles use fallbackMatId.
// Normals are generated per-face if absent from the OBJ.
static void loadObj(const std::string& filename, int fallbackMatId,
                    std::vector<Triangle>& tris,
                    std::vector<Material>* outMaterials = nullptr,
                    std::vector<Texture>*  outTextures  = nullptr,
                    std::unordered_map<std::string, int>* texCache = nullptr,
                    bool useMtl = false) {
    namespace fs = std::filesystem;

    tinyobj::ObjReaderConfig cfg;
    cfg.triangulate = true;
    if (useMtl)
        cfg.mtl_search_path = fs::path(filename).parent_path().string();

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

    // Build a mapping: OBJ material index → scene material index
    std::vector<int> matIdMap;
    if (useMtl && outMaterials) {
        const auto& mtlMats = reader.GetMaterials();
        std::string baseDir = fs::path(filename).parent_path().string();
        int base = (int)outMaterials->size();
        matIdMap.resize(mtlMats.size());
        for (int i = 0; i < (int)mtlMats.size(); i++) {
            const auto& m = mtlMats[i];
            Material mat;
            mat.albedo   = {m.diffuse[0],  m.diffuse[1],  m.diffuse[2]};
            mat.emission = {m.emission[0], m.emission[1], m.emission[2]};

            // Load diffuse texture if present
            if (!m.diffuse_texname.empty() && outTextures && texCache) {
                std::string texPath = (fs::path(baseDir) / m.diffuse_texname).string();
                auto it = texCache->find(texPath);
                if (it != texCache->end()) {
                    mat.albedoTex = it->second;
                } else {
                    Texture tex = Texture::loadFromFile(texPath);
                    if (tex.isValid()) {
                        int texIdx = (int)outTextures->size();
                        outTextures->push_back(std::move(tex));
                        (*texCache)[texPath] = texIdx;
                        mat.albedoTex = texIdx;
                    }
                }
            }

            outMaterials->push_back(mat);
            matIdMap[i] = base + i;
        }
    }

    for (const auto& shape : shapes) {
        size_t idxOff = 0;
        for (size_t f = 0; f < shape.mesh.num_face_vertices.size(); ++f) {
            int fv = (int)shape.mesh.num_face_vertices[f];
            if (fv != 3) { idxOff += fv; continue; }  // skip non-triangles

            // Resolve per-face material
            int triMatId = fallbackMatId;
            if (useMtl && !matIdMap.empty() && f < shape.mesh.material_ids.size()) {
                int mid = shape.mesh.material_ids[f];
                if (mid >= 0 && mid < (int)matIdMap.size())
                    triMatId = matIdMap[mid];
            }

            Triangle tri;
            tri.matId = triMatId;
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
                if (idx.texcoord_index >= 0) {
                    tri.uv[v][0] = attrib.texcoords[2 * idx.texcoord_index + 0];
                    // OBJ V=0 is bottom; stb_image V=0 is top → flip
                    tri.uv[v][1] = 1.0f - attrib.texcoords[2 * idx.texcoord_index + 1];
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
    std::unordered_map<std::string, int> texCache;  // filename → scene texture index

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
                bool useMtl = on["use_obj_materials"] && on["use_obj_materials"].as<bool>();
                loadObj(file, matId, scene->triangles, &scene->materials,
                        &scene->textures, &texCache, useMtl);

            } else if (type == "sphere") {
                Sphere sph;
                sph.matId   = matId;
                sph.radius  = on["radius"] ? on["radius"].as<float>() : 1.0f;
                if (on["center"])   sph.center   = parseVec3(on["center"]);
                if (on["raymarch"]) sph.raymarch = on["raymarch"].as<bool>();
                scene->spheres.push_back(sph);

            } else if (type == "torus") {
                Torus tor;
                tor.matId  = matId;
                tor.majorR = on["major_radius"] ? on["major_radius"].as<float>() : 1.0f;
                tor.minorR = on["minor_radius"]  ? on["minor_radius"].as<float>()  : 0.25f;
                if (on["center"]) tor.center = parseVec3(on["center"]);
                if (on["axis"])   tor.axis   = normalize(parseVec3(on["axis"]));
                scene->tori.push_back(tor);

            } else {
                std::fprintf(stderr, "[scene_loader] Unknown object type '%s'\n",
                             type.c_str());
            }
        }
    }

    // Build BVH over triangle soup
    scene->buildBVH();

    std::fprintf(stdout, "[scene_loader] Loaded '%s': %zu triangles, %zu spheres, %zu tori, %zu materials, %zu textures\n",
                 scene->name.c_str(),
                 scene->triangles.size(),
                 scene->spheres.size(),
                 scene->tori.size(),
                 scene->materials.size(),
                 scene->textures.size());

    return scene;
}

} // namespace PBRSceneLoader
