#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm> // For std::min and std::max
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <Eigen/Dense>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Custom clamp function for pre-C++17 compilers
template <typename T>
T clamp(const T& v, const T& lo, const T& hi) {
    return (v < lo) ? lo : (v > hi) ? hi : v;
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
        firstMouse = true; // Reset firstMouse to ignore sudden jumps
        return;
    }
    if (firstMouse) {
        lastX = static_cast<float>(xpos);
        lastY = static_cast<float>(ypos);
        firstMouse = false;
    }

    float xoffset = static_cast<float>(xpos) - lastX;
    float yoffset = lastY - static_cast<float>(ypos); // Reversed since y-coordinates go from bottom to top

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
static GLFWwindow* initGLFW()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return nullptr;
    }

    GLFWwindow* window = glfwCreateWindow(800, 600, "RBF Interpolation Visualization", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return nullptr;
    }

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync
    return window;
}

// Function to draw vertices
static void drawVertices(const Eigen::MatrixXd& vertices, const Eigen::Vector3f& color, float offset = 0.0f) {
    glColor3f(color.x(), color.y(), color.z());
    glBegin(GL_POINTS);
    for (Eigen::Index i = 0; i < vertices.rows(); ++i) {
        glVertex3f(static_cast<float>(vertices(i, 0)) + offset,
            static_cast<float>(vertices(i, 1)),
            static_cast<float>(vertices(i, 2)));
    }
    glEnd();
}

// Function to draw the axes
static void drawAxes() {
    glBegin(GL_LINES);

    // X axis in red
    glColor3f(1.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(1.0f, 0.0f, 0.0f);

    // Y axis in green
    glColor3f(0.0f, 1.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 1.0f, 0.0f);

    // Z axis in blue
    glColor3f(0.0f, 0.0f, 1.0f);
    glVertex3f(0.0f, 0.0f, 0.0f);
    glVertex3f(0.0f, 0.0f, 1.0f);

    glEnd();
}

// Compute initial weights (before applying epsilon) and initial offsets
static void computeInitialWeightsAndOffsets(const Eigen::MatrixXd& vertices, const Eigen::MatrixXd& controlPoints,
    Eigen::MatrixXd& weightsMatrixOrig, Eigen::MatrixXd& offsets)
{
    Eigen::Index numVertices = vertices.rows();
    Eigen::Index numControlPoints = controlPoints.rows();

    weightsMatrixOrig.resize(numVertices, numControlPoints);
    offsets.resize(numVertices, 3);

    for (Eigen::Index i = 0; i < numVertices; ++i) {
        Eigen::VectorXd distances(numControlPoints);
        for (Eigen::Index j = 0; j < numControlPoints; ++j) {
            distances(j) = (vertices.row(i) - controlPoints.row(j)).norm();
        }

        // Normalize weights
        double sumDistances = distances.sum();
        Eigen::VectorXd weights = distances / sumDistances;
        weightsMatrixOrig.row(i) = weights;

        // Compute initial offset
        Eigen::Vector3d originalVertex = weights.transpose() * controlPoints;
        offsets.row(i) = originalVertex.transpose() - vertices.row(i);
    }
}

// Update weights and offsets when epsilon changes
static void updateWeightsAndOffsets(const Eigen::MatrixXd& weightsMatrixOrig, double epsilon,
    const Eigen::MatrixXd& controlPoints,
    Eigen::MatrixXd& weightsMatrix, Eigen::MatrixXd& offsets,
    const Eigen::MatrixXd& vertices)
{
    Eigen::Index numVertices = weightsMatrixOrig.rows();
    Eigen::Index numControlPoints = weightsMatrixOrig.cols();

    weightsMatrix.resize(numVertices, numControlPoints);

    for (Eigen::Index i = 0; i < numVertices; ++i) {
        Eigen::VectorXd weights = weightsMatrixOrig.row(i);

        // Update weights with epsilon
        weights = (1.0 - weights.array().pow(0.01)).pow(epsilon);

        // Normalize weights
        double sumWeights = weights.sum();
        weights /= sumWeights;

        weightsMatrix.row(i) = weights;

        // Recompute offset with new weights
        Eigen::Vector3d originalVertex = weights.transpose() * controlPoints;
        offsets.row(i) = originalVertex.transpose() - vertices.row(i);
    }
}

// Apply RBF deformation with offset preservation
static Eigen::MatrixXd applyRBFDeformation(const Eigen::MatrixXd& weightsMatrix, const Eigen::MatrixXd& offsets,
    const Eigen::MatrixXd& deformedControlPoints)
{
    return (weightsMatrix * deformedControlPoints) - offsets;
}

// Generate a sphere mesh
static Eigen::MatrixXd generateSphere(int stacks, int slices, double radius) {
    std::vector<Eigen::Vector3d> vertices;
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
            vertices.emplace_back(radius * x, radius * y, radius * z);
        }
    }

    Eigen::MatrixXd sphereVertices(vertices.size(), 3);
    for (size_t i = 0; i < vertices.size(); ++i) {
        sphereVertices.row(static_cast<Eigen::Index>(i)) = vertices[i];
    }
    return sphereVertices;
}

int main()
{
    // Initialize GLFW
    GLFWwindow* window = initGLFW();
    if (!window)
        return -1;

    // Set mouse and scroll callbacks
    glfwSetCursorPosCallback(window, mouse_callback);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 130");

    // Generate a sphere
    Eigen::MatrixXd sphereVertices = generateSphere(20, 20, 1.0);

    // Define control points
    Eigen::MatrixXd controlPoints(12, 3);
    controlPoints << -1, -1, -1,
        1, -1, -1,
        -1, 1, -1,
        1, 1, -1,
        -1, -1, 1,
        1, -1, 1,
        -1, 1, 1,
        1, 1, 1,
        -1, 0, -1,
        1, 0, 1,
        -1, 0, 1,
        1, 0, -1;

    // Deformed control points (initially same as control points)
    Eigen::MatrixXd deformedControlPoints = controlPoints;

    // Parameters
    float epsilon = 8.0f;
    float prev_epsilon = epsilon;

    // Precompute initial weights and offsets
    Eigen::MatrixXd weightsMatrixOrig, offsets;
    computeInitialWeightsAndOffsets(sphereVertices, controlPoints, weightsMatrixOrig, offsets);

    Eigen::MatrixXd weightsMatrix = weightsMatrixOrig;

    // Update weights and offsets with initial epsilon
    updateWeightsAndOffsets(weightsMatrixOrig, epsilon, controlPoints, weightsMatrix, offsets, sphereVertices);

    Eigen::MatrixXd deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        glfwPollEvents();

        // Start the ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // Initialize controlPointsChanged flag
        bool controlPointsChanged = false;

        // ImGui controls for epsilon and control points
        ImGui::Begin("RBF Interpolation Controls");
        if (ImGui::SliderFloat("Epsilon", &epsilon, 0.1f, 100.0f)) {
            if (std::abs(epsilon - prev_epsilon) > 0.0001f) {
                updateWeightsAndOffsets(weightsMatrixOrig, epsilon, controlPoints, weightsMatrix, offsets, sphereVertices);
                deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);
                prev_epsilon = epsilon;
            }
        }
        // Add Reset Button
        if (ImGui::Button("Reset Control Points")) {
            deformedControlPoints = controlPoints; // Reset to original positions
            controlPointsChanged = true; // Indicate that control points have changed
        }

        //bool controlPointsChanged = false;
        for (Eigen::Index i = 0; i < controlPoints.rows(); ++i) {
            float cp[3] = { static_cast<float>(deformedControlPoints(i, 0)),
                            static_cast<float>(deformedControlPoints(i, 1)),
                            static_cast<float>(deformedControlPoints(i, 2)) };
            if (ImGui::SliderFloat3(("Control Point " + std::to_string(static_cast<int>(i))).c_str(), cp, -10.0f, 10.0f)) {
                deformedControlPoints.row(i) << cp[0], cp[1], cp[2];
                controlPointsChanged = true;
            }
        }
        ImGui::End();

        // Recompute deformed vertices if control points changed
        if (controlPointsChanged) {
            deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);
        }
        // Recompute deformed vertices if control points changed
        if (controlPointsChanged) {
            deformedVertices = applyRBFDeformation(weightsMatrix, offsets, deformedControlPoints);
        }
        // Render
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Set up camera using GLM
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

        // Render control points
        glPointSize(10);
        drawVertices(deformedControlPoints, Eigen::Vector3f(1.0f, 0.0f, 0.0f)); // Red

        // Render deformed sphere
        glPointSize(1);
        drawVertices(deformedVertices, Eigen::Vector3f(0.0f, 1.0f, 0.0f)); // Green

        drawAxes();

        // Render ImGui
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
    return 0;
}
