#pragma once
#include <cstdint>
constexpr float PI = 3.14159f;
constexpr float RAD = PI/180;
constexpr const char* RES_PATH = "resources/";
constexpr const char* SCENES_PATH = "resources/scenes";  
constexpr int SHADOW_MAP_SIZE = 512;
constexpr float SHADOW_BIAS_MIN = 0.02f;
constexpr float SHADOW_BIAS_MAX = 0.25f;
constexpr int SHADOW_PCF_RADIUS = 0;
constexpr int SHADOW_MAP_OVERVIEW_SIZE = 200;
constexpr float EFFECTIVE_LIGHT_RADIUS_FACTOR = 1.00f;

constexpr float MIN_BIAS_DEFAULT = 0.025f;
constexpr float MAX_BIAS_DEFAULT = 0.05f;
constexpr float CUBE_SHADOW_MAX_SLOPE_BIAS = 10.0f;

constexpr float CAMERA_DEFAULT_ZNEAR = 10.0f;
constexpr float CAMERA_DEFAULT_ZFAR = 10000.0f;
constexpr float CAMERA_DEFAULT_VIEW_ANGLE = 45.0f;

constexpr int SCREEN_WIDTH = 640;
constexpr int SCREEN_HEIGHT = 480;

constexpr uint32_t RED_COLOR = 0xffff0000;
constexpr uint32_t GREEN_COLOR = 0xff00ff00;
constexpr uint32_t BLUE_COLOR = 0xff0000ff;
constexpr uint32_t WHITE_COLOR = 0xffffffff;
constexpr uint32_t BLACK_COLOR = 0xff000000;
constexpr uint32_t YELLOW_COLOR = 0xff00ffff;
constexpr uint32_t GREY_COLOR = 0xff404040;
constexpr uint32_t CLEAR_GREY_COLOR = 0xffc0c0c0;
