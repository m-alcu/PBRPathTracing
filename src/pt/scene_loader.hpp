#pragma once
#include "scene.hpp"
#include <memory>
#include <string>

namespace PBRSceneLoader {

// Load a PBRScene from a YAML file.
// Objects are loaded via tinyobjloader; materials from the YAML.
// Calls scene.buildBVH() before returning.
std::unique_ptr<PBRScene> loadFromFile(const std::string& yamlPath);

// Save a PBRScene to a YAML file that can be re-loaded by loadFromFile.
// Material names are generated as "mat_0", "mat_1", … since PBRScene does
// not store them.  Triangles sourced from OBJ files are NOT written (they
// would need to be re-added as "type: obj" entries by hand).
void saveToFile(const PBRScene& scene, const std::string& yamlPath);

} // namespace PBRSceneLoader
