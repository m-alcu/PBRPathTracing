#pragma once
#include "scene.hpp"
#include <memory>
#include <string>

namespace PBRSceneLoader {

// Load a PBRScene from a YAML file.
// Objects are loaded via tinyobjloader; materials from the YAML.
// Calls scene.buildBVH() before returning.
std::unique_ptr<PBRScene> loadFromFile(const std::string& yamlPath);

} // namespace PBRSceneLoader
