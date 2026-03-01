cmake_minimum_required(VERSION 3.28)
project(PBRPathTracing)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

include(FetchContent)
set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

# SDL3
FetchContent_Declare(
    SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.4.0
    GIT_SHALLOW FALSE
)
FetchContent_MakeAvailable(SDL3)

# yaml-cpp
FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.9.0
)
set(YAML_CPP_BUILD_TESTS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_TOOLS   OFF CACHE BOOL "" FORCE)
set(YAML_CPP_BUILD_CONTRIB OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(yaml-cpp)

# ---------------------------------------------------------------------------
# Main executable – PBR Path Tracer
# ---------------------------------------------------------------------------

set(PT_SOURCES
    src/main.cpp
    src/pt/scene_loader.cpp
    # Dear ImGui
    src/vendor/imgui/imgui.cpp
    src/vendor/imgui/imgui_draw.cpp
    src/vendor/imgui/imgui_tables.cpp
    src/vendor/imgui/imgui_widgets.cpp
    src/vendor/imgui/imgui_impl_sdl3.cpp
    src/vendor/imgui/imgui_impl_sdlrenderer3.cpp
    # stb_image (header-only impl unit)
    src/vendor/nothings/stb_image.cpp
    # tinyobjloader
    src/vendor/tinyobjloader/tiny_obj_loader.cpp
)

add_executable(${PROJECT_NAME} ${PT_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_link_libraries(${PROJECT_NAME} PRIVATE
    SDL3::SDL3
    yaml-cpp::yaml-cpp
)

target_compile_options(${PROJECT_NAME} PRIVATE
    $<$<CONFIG:Release>:-O3>
)

set_target_properties(${PROJECT_NAME} PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)

# Copy resources to binary directory
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/resources
    $<TARGET_FILE_DIR:${PROJECT_NAME}>/resources
)

# ---------------------------------------------------------------------------
# Tests (math / transform – they only depend on slib + smath)
# ---------------------------------------------------------------------------

option(BUILD_TESTS "Build unit tests" ON)

if(BUILD_TESTS)
    enable_testing()

    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.14.0
    )
    set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)

    # Math library test
    add_executable(test_math
        tests/test_math.cpp
        src/slib.cpp
        src/smath.cpp
    )
    target_include_directories(test_math PRIVATE ${CMAKE_SOURCE_DIR}/src)
    target_link_libraries(test_math PRIVATE GTest::gtest_main)
    set_target_properties(test_math PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)

    include(GoogleTest)
    gtest_discover_tests(test_math)
endif()
