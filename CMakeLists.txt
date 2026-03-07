cmake_minimum_required(VERSION 3.28)
project(PBRPathTracing)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

set(CMAKE_POLICY_VERSION_MINIMUM 3.5)

# ---------------------------------------------------------------------------
# Dependencies
# ---------------------------------------------------------------------------

find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)

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
    src/vendor/imgui/imgui_impl_glfw.cpp
    src/vendor/imgui/imgui_impl_opengl3.cpp
    # stb_image (header-only impl unit)
    src/vendor/nothings/stb_image.cpp
    # tinyobjloader
    src/vendor/tinyobjloader/tiny_obj_loader.cpp
)

add_executable(${PROJECT_NAME} ${PT_SOURCES})

target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_SOURCE_DIR}/src)

target_compile_definitions(${PROJECT_NAME} PRIVATE
    IMGUI_IMPL_OPENGL_LOADER_CUSTOM
)

target_link_libraries(${PROJECT_NAME} PRIVATE
    glfw
    OpenGL::GL
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

    add_executable(test_math
        tests/test_math.cpp
        src/slib.cpp
        src/smath.cpp
    )
    target_include_directories(test_math PRIVATE ${CMAKE_SOURCE_DIR}/src)
    set_target_properties(test_math PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    add_test(NAME test_math COMMAND test_math)

    add_executable(test_pt
        tests/test_pt.cpp
        src/vendor/nothings/stb_image.cpp
    )
    target_include_directories(test_pt PRIVATE ${CMAKE_SOURCE_DIR}/src)
    set_target_properties(test_pt PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    add_test(NAME test_pt COMMAND test_pt)
endif()
