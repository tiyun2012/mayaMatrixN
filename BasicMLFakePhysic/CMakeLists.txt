cmake_minimum_required(VERSION 3.15)

# Declare the project name
set(PROJECT_NAME BasicML)
project(${PROJECT_NAME} LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configurations
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Build configurations" FORCE)

# Option to enable/disable Maya plugin build
option(BUILD_MAYA_PLUGIN "Build the Maya plugin (MLVertexBaseTranform.mll)" OFF)

# Include directories for visualization tool
include_directories(
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/eigen-master"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/glfw-3.4/include"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/backends"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/glad/include"
)

# ImGui sources
set(IMGUI_SOURCES
    E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/imgui.cpp
    E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/imgui_draw.cpp
    E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/imgui_widgets.cpp
    E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/imgui_tables.cpp
    E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/backends/imgui_impl_glfw.cpp
    E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/backends/imgui_impl_opengl3.cpp
)

# GLAD sources
set(GLAD_SOURCES
    glad/src/glad.c
)
set_source_files_properties(${GLAD_SOURCES} PROPERTIES LANGUAGE C)

# Visualization executable
add_executable(VisualizePoints visualize_points.cpp ${IMGUI_SOURCES} ${GLAD_SOURCES})
target_include_directories(VisualizePoints PRIVATE
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/eigen-master"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/glfw-3.4/include"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/imgui/backends"
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/glad/include"
)
target_link_directories(VisualizePoints PRIVATE
    "E:/dev/RBF/pointsData/BasicMLFakePhysic/glfw-3.4/lib-vc2022"
)
target_link_libraries(VisualizePoints PRIVATE
    glfw3.lib
    opengl32.lib
)

# Debug configuration for VisualizePoints
target_compile_definitions(VisualizePoints PRIVATE $<$<CONFIG:Debug>:_DEBUG>)
target_compile_options(VisualizePoints PRIVATE $<$<CONFIG:Debug>:/W3 /SDL>)
target_link_options(VisualizePoints PRIVATE $<$<CONFIG:Debug>:/DEBUG>)

# Release configuration for VisualizePoints
target_compile_definitions(VisualizePoints PRIVATE $<$<CONFIG:Release>:NDEBUG>)
target_compile_options(VisualizePoints PRIVATE $<$<CONFIG:Release>:/W3 /SDL /O2>)
target_link_options(VisualizePoints PRIVATE $<$<CONFIG:Release>:/OPT:REF /OPT:ICF>)

# Maya plugin (optional)
if(BUILD_MAYA_PLUGIN)
    # Include directories for Maya plugin
    include_directories(
        "C:/Program Files/Autodesk/Maya2025"
        "C:/Program Files/Autodesk/Maya2025/include"
        "E:/dev/RBF/devkit2025/include"
    )

    # Source files for plugin
    set(SOURCES
        mlVertiesBaseTransform.cpp
    )

    # Add the library (DynamicLibrary for .mll output)
    add_library(${PROJECT_NAME} MODULE ${SOURCES})

    # Set target properties
    set_target_properties(${PROJECT_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_BINARY_DIR}/Debug
        RUNTIME_OUTPUT_DIRECTORY_RELEASE ${CMAKE_BINARY_DIR}/Release
        OUTPUT_NAME MLVertexBaseTranform
        SUFFIX ".mll"
    )

    # Platform-specific settings
    if(WIN32)
        target_compile_definitions(${PROJECT_NAME} PRIVATE WIN64 _WINDOWS NT_PLUGIN REQUIRE_IOSTREAM)
    endif()

    # Library directories
    target_link_directories(${PROJECT_NAME} PRIVATE
        "C:/Program Files/Autodesk/Maya2025/lib"
        "E:/dev/RBF/devkit2025/lib"
    )

    # Link dependencies
    target_link_libraries(${PROJECT_NAME} PRIVATE
        OpenMayaUI.lib
        OpenMaya.lib
        Foundation.lib
        OpenMayaFX.lib
        OpenMayaAnim.lib
    )

    # Debug configuration
    target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:_DEBUG>)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:/W3 /SDL>)
    target_link_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Debug>:/DEBUG>)

    # Release configuration
    target_compile_definitions(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:NDEBUG>)
    target_compile_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:/W3 /SDL /O2>)
    target_link_options(${PROJECT_NAME} PRIVATE $<$<CONFIG:Release>:/OPT:REF /OPT:ICF>)
endif()