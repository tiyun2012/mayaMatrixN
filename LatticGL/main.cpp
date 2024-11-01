#include <iostream>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>
#include <sstream>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Custom clamp function for pre-C++17 compilers
template <typename T>
T clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

// Custom to_string function for compatibility with pre-C++11 compilers
template <typename T>
std::string to_string(const T& value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// Camera control variables
float cameraDistance = 5.0f;
float cameraAngleX = 0.0f;
float cameraAngleY = 0.0f;
float lastX = 400, lastY = 300;
bool firstMouse = true;
float panX = 0.0f, panY = 0.0f;
bool leftMouseButtonPressed = false;
bool rightMouseButtonPressed = false;

// Mouse callback function
static void mouse_callback(GLFWwindow* window, double xpos, double ypos) {
    if (ImGui::GetIO().WantCaptureMouse) {
        firstMouse = true;
        return;
    }
    if (firstMouse) {
        lastX = static_cast<float>(xpos);
        lastY = static_cast<float>(ypos);
        firstMouse = false;
    }
    float xoffset = static_cast<float>(xpos) - lastX;
    float yoffset = lastY - static_cast<float>(ypos);
    lastX = static_cast<float>(xpos);
    lastY = static_cast<float>(ypos);

    float sensitivity = 0.1f;
    xoffset *= sensitivity;
    yoffset *= sensitivity;

    if (leftMouseButtonPressed) {
        cameraAngleY += xoffset;
        cameraAngleX += yoffset;
        cameraAngleX = clamp(cameraAngleX, -89.0f, 89.0f);
    }
    if (rightMouseButtonPressed) {
        panX += xoffset * 0.1f;
        panY += yoffset * 0.1f;
    }
}

// Mouse button callback function
static void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        leftMouseButtonPressed = (action == GLFW_PRESS);
    }
    if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        rightMouseButtonPressed = (action == GLFW_PRESS);
    }
}

// Scroll callback function
static void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    if (ImGui::GetIO().WantCaptureMouse)
        return;
    cameraDistance -= static_cast<float>(yoffset);
    cameraDistance = clamp(cameraDistance, 1.0f, 100.0f);
}

// Function to initialize GLFW and create a window
static GLFWwindow* initGLFW() {
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return nullptr;
    }
    GLFWwindow* window = glfwCreateWindow(800, 600, "RBF Interpolation Visualization", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    return window;
}

// Helper function to initialize a 2D vector with dimensions (rows, cols)
std::vector<std::vector<double>> create2DVector(int rows, int cols, double initialValue = 0.0) {
    return std::vector<std::vector<double>>(rows, std::vector<double>(cols, initialValue));
}

// Helper function to normalize a vector
std::vector<double> normalizeVector(const std::vector<double>& vec) {
    double sum = std::accumulate(vec.begin(), vec.end(), 0.0);
    std::vector<double> result(vec.size());
    if (sum != 0.0) {
        for (size_t i = 0; i < vec.size(); ++i) {
            result[i] = vec[i] / sum;
        }
    }
    return result;
}

// Function to compute initial weights and offsets
static void computeInitialWeightsAndOffsets(const std::vector<std::vector<double>>& vertices,
                                            const std::vector<std::vector<double>>& controlPoints,
                                            std::vector<std::vector<double>>& weightsMatrixOrig,
                                            std::vector<std::vector<double>>& offsets) {
    int numVertices = vertices.size();
    int numControlPoints = controlPoints.size();
    weightsMatrixOrig = create2DVector(numVertices, numControlPoints);
    offsets = create2DVector(numVertices, 3);

    for (int i = 0; i < numVertices; ++i) {
        std::vector<double> distances(numControlPoints);
        for (int j = 0; j < numControlPoints; ++j) {
            double dist = 0.0;
            for (int k = 0; k < 3; ++k) {
                dist += (vertices[i][k] - controlPoints[j][k]) * (vertices[i][k] - controlPoints[j][k]);
            }
            distances[j] = sqrt(dist);
        }
        std::vector<double> weights = normalizeVector(distances);
        weightsMatrixOrig[i] = weights;

        std::vector<double> originalVertex(3, 0.0);
        for (int j = 0; j < numControlPoints; ++j) {
            for (int k = 0; k < 3; ++k) {
                originalVertex[k] += weights[j] * controlPoints[j][k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            offsets[i][k] = originalVertex[k] - vertices[i][k];
        }
    }
}

// Function to update weights and offsets with epsilon
static void updateWeightsAndOffsets(const std::vector<std::vector<double>>& weightsMatrixOrig, double epsilon,
                                    const std::vector<std::vector<double>>& controlPoints,
                                    std::vector<std::vector<double>>& weightsMatrix,
                                    std::vector<std::vector<double>>& offsets,
                                    const std::vector<std::vector<double>>& vertices) {
    int numVertices = weightsMatrixOrig.size();
    int numControlPoints = weightsMatrixOrig[0].size();
    weightsMatrix = create2DVector(numVertices, numControlPoints);

    for (int i = 0; i < numVertices; ++i) {
        std::vector<double> weights = weightsMatrixOrig[i];
        for (auto& w : weights) {
            w = pow(1.0 - pow(w, 0.01), epsilon);
        }
        weights = normalizeVector(weights);
        weightsMatrix[i] = weights;

        std::vector<double> originalVertex(3, 0.0);
        for (int j = 0; j < numControlPoints; ++j) {
            for (int k = 0; k < 3; ++k) {
                originalVertex[k] += weights[j] * controlPoints[j][k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            offsets[i][k] = originalVertex[k] - vertices[i][k];
        }
    }
}

// Apply RBF deformation with offset preservation
static std::vector<std::vector<double>> applyRBFDeformation(const std::vector<std::vector<double>>& weightsMatrix,
                                                            const std::vector<std::vector<double>>& offsets,
                                                            const std::vector<std::vector<double>>& deformedControlPoints) {
    int numVertices = weightsMatrix.size();
    std::vector<std::vector<double>> deformedVertices = create2DVector(numVertices, 3);

    for (int i = 0; i < numVertices; ++i) {
        std::vector<double> deformedVertex(3, 0.0);
        for (int j = 0; j < weightsMatrix[i].size(); ++j) {
            for (int k = 0; k < 3; ++k) {
                deformedVertex[k] += weightsMatrix[i][j] * deformedControlPoints[j][k];
            }
        }
        for (int k = 0; k < 3; ++k) {
            deformedVertices[i][k] = deformedVertex[k] - offsets[i][k];
        }
    }
    return deformedVertices;
}

// Generate a sphere mesh
static std::vector<std::vector<double>> generateSphere(int stacks, int slices, double radius) {
    std::vector<std::vector<double>> vertices;
    for (int i = 0; i <= stacks; ++i) {
        double lat = M_PI * (-0.5 + static_cast<double>(i) / stacks);
        double sinLat = sin(lat);
        double cosLat = cos(lat);

        for (int j = 0; j <= slices; ++j) {
            double lon = 2 * M_PI * static_cast<double>(j) / slices;
            double sinLon = sin(lon);
            double cosLon = cos(lon);

            double x = cosLon * cosLat;
            double y = sinLat;
            double z = sinLon * cosLat;
            vertices.push_back({ radius * x, radius * y, radius * z });
        }
    }
    return vertices;
}

int main() {
    GLFWwindow* window = initGLFW();
    if (!window) return -1;

    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    std::vector<std::vector<double>> sphereVertices = generateSphere(20, 20, 1.0);
    std::vector<std::vector<double>> controlPoints = {{-1, -1, -1}, {1, -1, -1}, {-1, 1, -1}, {1, 1, -1}, {-1, -1, 1}, 
                                                      {1, -1, 1}, {-1, 1, 1}, {1, 1, 1}, {-1, 0, -1}, {1, 0, 1}, 
                                                      {-1, 0, 1}, {1, 0, -1}};
    std::vector<std::vector<double>> deformedControlPoints = controlPoints;
    float epsilon = 8.0f, prev_epsilon = epsilon;

    std::vector<std::vector<double>> weightsMatrixOrig, offsets;
    computeInitialWeightsAndOffsets(sphereVertices, controlPoints, weightsMatrixOrig, offsets);
    std::vector<std::vector<double>> weightsMatrix = weightsMatrixOrig;

    updateWeightsAndOffsets(weightsMatrixOrig, epsilon, controlPoints, weightsMatrix, offsets, sphereVertices);
    std::vector<std::vector<double>> deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        bool controlPointsChanged = false;

        ImGui::Begin("RBF Interpolation Controls");
        if (ImGui::SliderFloat("Epsilon", &epsilon, 0.1f, 100.0f)) {
            if (std::abs(epsilon - prev_epsilon) > 0.0001f) {
                updateWeightsAndOffsets(weightsMatrixOrig, epsilon, controlPoints, weightsMatrix, offsets, sphereVertices);
                deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);
                prev_epsilon = epsilon;
            }
        }
        if (ImGui::Button("Reset Control Points")) {
            deformedControlPoints = controlPoints;
            controlPointsChanged = true;
        }
        for (size_t i = 0; i < controlPoints.size(); ++i) {
            float cp[3] = { static_cast<float>(deformedControlPoints[i][0]), 
                            static_cast<float>(deformedControlPoints[i][1]), 
                            static_cast<float>(deformedControlPoints[i][2]) };
            if (ImGui::SliderFloat3(("Control Point " + to_string(i)).c_str(), cp, -10.0f, 10.0f)) {
                deformedControlPoints[i] = { cp[0], cp[1], cp[2] };
                controlPointsChanged = true;
            }
        }
        ImGui::End();
        
        if (controlPointsChanged) {
            deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glm::mat4 projection = glm::perspective(glm::radians(45.0f), 800.0f / 600.0f, 0.1f, 100.0f);
        glm::mat4 view = glm::lookAt(
            glm::vec3(cameraDistance * sin(glm::radians(cameraAngleY)) * cos(glm::radians(cameraAngleX)),
                      cameraDistance * sin(glm::radians(cameraAngleX)),
                      cameraDistance * cos(glm::radians(cameraAngleY)) * cos(glm::radians(cameraAngleX))),
            glm::vec3(panX, panY, 0.0f),
            glm::vec3(0.0f, 1.0f, 0.0f));
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf(glm::value_ptr(projection));
        glMatrixMode(GL_MODELVIEW);
        glLoadMatrixf(glm::value_ptr(view));

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
        glfwSwapBuffers(window);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
