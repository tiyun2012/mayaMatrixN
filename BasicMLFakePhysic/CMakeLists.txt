cmake_minimum_required(VERSION 3.15)

# Declare the project name explicitly
set(PROJECT_NAME BasicML)

# Use the project name
project(${PROJECT_NAME} LANGUAGES CXX)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configurations
set(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE STRING "Build configurations" FORCE)

# Include directories
include_directories(
    "C:/Program Files/Autodesk/Maya2025"
    "C:/Program Files/Autodesk/Maya2025/include"
    "E:/dev/RBF/devkit2025/include"
    "E:/dev/RBF/pointsData/Matrix"
    "E:/dev/RBF/eigen-master"
    "E:/dev/RBF/nanoflann-master/include"
)

# Source files
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
