// GLAD: OpenGL function loader for modern OpenGL (3.3 core profile).
// Benefit: Loads OpenGL function pointers at runtime, enabling use of functions like glGenVertexArrays, glBindBuffer, etc.
//          Required because GLFW only provides an OpenGL context, not function definitions. GLAD ensures compatibility
//          with your OpenGL version and avoids manual function pointer management.
#include <glad/glad.h>

// GLFW: Cross-platform library for creating windows, handling OpenGL contexts, and managing input (keyboard, mouse).
// Benefit: Simplifies window creation and OpenGL context setup, allowing focus on rendering. Provides event handling for
//          user interaction (e.g., closing the window). Essential for rendering the 3D point cloud in a window.
#include <GLFW/glfw3.h>

// ImGui: Immediate-mode GUI library for creating user interfaces (e.g., sliders, checkboxes).
// Benefit: Enables quick creation of a control panel with a frame slider and "Show Normalized" checkbox. Lightweight and
//          integrates seamlessly with GLFW and OpenGL, making it ideal for debugging and interactive data visualization.
#include <imgui.h>

// ImGui GLFW Backend: Connects ImGui with GLFW for input handling and window integration.
// Benefit: Allows ImGui to process GLFW input events (e.g., mouse clicks on sliders). Ensures the GUI responds to user
//          interactions within the GLFW window, critical for the frame slider functionality.
#include <imgui_impl_glfw.h>

// ImGui OpenGL Backend: Renders ImGui widgets using OpenGL.
// Benefit: Draws ImGui’s GUI elements (e.g., sliders, buttons) on the OpenGL context. Ensures the control panel is
//          rendered alongside the 3D point cloud, providing a unified visualization interface.
#include <imgui_impl_opengl3.h>

// Eigen: High-performance C++ library for linear algebra and matrix operations.
// Benefit: Handles matrix operations for data processing (e.g., normalizing point coordinates, computing MVP matrices).
//          Its template-based design is efficient and integrates well with your existing project (used in training code).
#include <Eigen/Dense>

// fstream: C++ standard library for file input/output operations.
// Benefit: Enables reading training_data.csv to load frame data (transform matrices and points). Essential for accessing
//          the dataset that drives the visualization.
#include <fstream>

// sstream: C++ standard library for string stream operations.
// Benefit: Facilitates parsing CSV lines by splitting comma-separated values into doubles. Simplifies data extraction
//          from training_data.csv.
#include <sstream>

// vector: C++ standard library container for dynamic arrays.
// Benefit: Stores frame data (transform matrices, points) in a resizable container, allowing flexible handling of
//          variable numbers of frames from the CSV file.
#include <vector>

// string: C++ standard library for string manipulation.
// Benefit: Manages file paths and CSV parsing (e.g., extracting tokens). Necessary for handling file I/O and error messages.
#include <string>

// iostream: C++ standard library for console input/output.
// Benefit: Provides console logging for debugging (e.g., number of frames loaded, point coordinates). Helps verify data
//          parsing and rendering issues during development.
#include <iostream>

struct Camera {
    float theta = 0.0f;    // Azimuthal angle (radians)
    float phi = 0.0f;      // Polar angle (radians)
    float radius = 5.0f;   // Distance from origin
    Eigen::Vector3f target = Eigen::Vector3f::Zero(); // Look-at point
    bool isRotating = false;
    bool isPanning = false;
    double lastX = 0.0;
    double lastY = 0.0;

    // Compute view matrix
    Eigen::Matrix4f getViewMatrix() const {
        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        Eigen::Vector3f eye;
        eye.x() = target.x() + radius * sinf(phi) * cosf(theta);
        eye.y() = target.y() + radius * cosf(phi);
        eye.z() = target.z() + radius * sinf(phi) * sinf(theta);

        // Look-at matrix
        Eigen::Vector3f forward = (target - eye).normalized();
        Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f right = forward.cross(up).normalized();
        up = right.cross(forward).normalized();

        Eigen::Matrix4f lookAt = Eigen::Matrix4f::Identity();
        lookAt.block<1,3>(0,0) = right.transpose();
        lookAt.block<1,3>(1,0) = up.transpose();
        lookAt.block<1,3>(2,0) = -forward.transpose();
        lookAt(0,3) = -right.dot(eye);
        lookAt(1,3) = -up.dot(eye);
        lookAt(2,3) = forward.dot(eye);
        return lookAt;
    }
};
struct FrameData {
    Eigen::MatrixXd transform; // 4x4 transform matrix (double for precision in calculations)
    Eigen::MatrixXf points;    // 98x3 raw points (float for OpenGL compatibility)
    Eigen::MatrixXf points_normalized; // 98x3 normalized points
};

std::vector<FrameData> parseCSV(const std::string& filePath, Eigen::VectorXf& inputMean, Eigen::VectorXf& inputStd,
                                Eigen::VectorXf& outputMean, Eigen::VectorXf& outputStd) {
    std::vector<FrameData> data;
    std::ifstream file(filePath);
    std::string line;

    if (!file.is_open()) {
        std::cerr << "Failed to open CSV file: " << filePath << std::endl;
        return data;
    }

    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<double> values;

        while (std::getline(ss, token, ',')) {
            try {
                values.push_back(std::stod(token));
            } catch (...) {
                std::cerr << "Invalid value in CSV: " << token << std::endl;
                continue;
            }
        }

        if (values.size() != 311) {
            std::cerr << "Invalid row size: " << values.size() << ", expected 311" << std::endl;
            continue;
        }

        FrameData frame;
        frame.transform.resize(4, 4);
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                frame.transform(i, j) = values[1 + i * 4 + j];
            }
        }

        frame.points.resize(98, 3);
        for (int i = 0; i < 98; ++i) {
            frame.points(i, 0) = static_cast<float>(values[17 + i * 3]);
            frame.points(i, 1) = static_cast<float>(values[17 + i * 3 + 1]);
            frame.points(i, 2) = static_cast<float>(values[17 + i * 3 + 2]);
        }

        data.push_back(frame);
    }

    file.close();

    // Normalize data
    Eigen::MatrixXf allPoints(data.size() * 98, 3);
    for (size_t i = 0; i < data.size(); ++i) {
        for (int j = 0; j < 98; ++j) {
            allPoints.row(i * 98 + j) = data[i].points.row(j);
        }
    }

    inputMean = Eigen::VectorXf::Zero(3); // Placeholder (not used for points)
    inputStd = Eigen::VectorXf::Ones(3);
    outputMean = allPoints.colwise().mean();
    outputStd = ((allPoints.rowwise() - outputMean.transpose()).array().square().colwise().sum() / (allPoints.rows() - 1)).sqrt();
    for (int i = 0; i < outputStd.size(); ++i) {
        if (outputStd(i) < 1e-6) outputStd(i) = 1.0;
    }

    for (size_t i = 0; i < data.size(); ++i) {
        data[i].points_normalized.resize(98, 3);
        for (int j = 0; j < 98; ++j) {
            data[i].points_normalized.row(j) = (data[i].points.row(j) - outputMean.transpose()).array() / outputStd.transpose().array();
        }
    }

    std::cout << "Loaded " << data.size() << " frames with 98 points each." << std::endl;
    return data;
}

void checkGLError(const char* operation) {
    GLenum err = glGetError();
    if (err != GL_NO_ERROR) {
        std::cerr << "OpenGL error after " << operation << ": " << err << std::endl;
    }
}
void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        ImGuiIO& io = ImGui::GetIO();
        if (io.WantCaptureMouse) {
            std::cout << "Control panel context" << std::endl;
        } else {
            std::cout << "Viewport context" << std::endl;
        }
    }
}
int main() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    // Create window with OpenGL 3.3 core profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1280, 720, "Point Cloud Visualizer", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    glfwSetMouseButtonCallback(window, mouse_button_callback); // Added callback

    // Initialize GLAD to load OpenGL functions
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        std::cerr << "Failed to initialize GLAD" << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Initialize ImGui
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    ImGui::StyleColorsDark();
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");
    // camera setup
    Camera camera;
    // Load data
    Eigen::VectorXf inputMean, inputStd, outputMean, outputStd;
    std::vector<FrameData> data = parseCSV("E:/dev/RBF/pointsData/BasicMLFakePhysic/data/training_data.csv", inputMean, inputStd, outputMean, outputStd);
    if (data.empty()) {
        std::cerr << "No data loaded, exiting." << std::endl;
        glfwDestroyWindow(window);
        glfwTerminate();
        return -1;
    }

    // Initialize OpenGL buffers
    GLuint vbo, vao;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glBindVertexArray(vao);
    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    checkGLError("VAO/VBO setup");

    // Preload all frames into CPU memory for fast VBO updates
    std::vector<std::vector<float>> frameBuffers(data.size());
    for (size_t i = 0; i < data.size(); ++i) {
        frameBuffers[i].resize(98 * 3);
        for (int j = 0; j < 98; ++j) {
            frameBuffers[i][j * 3] = data[i].points(j, 0);
            frameBuffers[i][j * 3 + 1] = data[i].points(j, 1);
            frameBuffers[i][j * 3 + 2] = data[i].points(j, 2);
        }
    }

    // Vertex and fragment shaders
    const char* vertexShaderSource = R"(
        #version 330 core
        layout(location = 0) in vec3 aPos;
        uniform mat4 mvp;
        void main() {
            gl_Position = mvp * vec4(aPos, 1.0);
            gl_PointSize = 5.0;
        }
    )";
    const char* fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        void main() {
            FragColor = vec4(1.0, 1.0, 0.0, 1.0); // Yellow
        }
    )";

    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);
    GLint success;
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        std::cerr << "Vertex shader compilation failed: " << infoLog << std::endl;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        std::cerr << "Fragment shader compilation failed: " << infoLog << std::endl;
    }

    GLuint shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char infoLog[512];
        glGetProgramInfoLog(shaderProgram, 512, nullptr, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
    }
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // Camera and projection
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float aspect = 1280.0f / 720.0f;
    projection(0, 0) = 1.0f / (aspect * tanf(45.0f * 3.1415926535f / 360.0f));
    projection(1, 1) = 1.0f / tanf(45.0f * 3.1415926535f / 360.0f);
    projection(2, 2) = -(1000.0f + 0.1f) / (1000.0f - 0.1f);
    projection(2, 3) = -2.0f * 1000.0f * 0.1f / (1000.0f - 0.1f);
    projection(3, 2) = -1.0f;
    projection(3, 3) = 0.0f;

    // Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    // view(2, 3) = -5.0f; // Move camera back

    int currentFrame = 0;
    bool showNormalized = false;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        ImGui::Begin("Control Panel");
        ImGui::SliderInt("Frame", &currentFrame, 0, static_cast<int>(data.size()) - 1);
        ImGui::Checkbox("Show Normalized", &showNormalized);
        ImGui::Text("Frame %d: Translation (%.2f, %.2f, %.2f)", currentFrame,
                    data[currentFrame].transform(3, 0), data[currentFrame].transform(3, 1), data[currentFrame].transform(3, 2));
        if (ImGui::Button("Log Points")) {
            const auto& points = showNormalized ? data[currentFrame].points_normalized : data[currentFrame].points;
            std::cout << "Frame " << currentFrame << " Points:\n";
            for (int j = 0; j < 98; ++j) {
                std::cout << j << ": (" << points(j, 0) << ", " << points(j, 1) << ", " << points(j, 2) << ")\n";
            }
        }
        ImGui::End();

        // Update VBO
        std::vector<float> renderPoints(98 * 3);
        for (int j = 0; j < 98; ++j) {
            const auto& points = showNormalized ? data[currentFrame].points_normalized : data[currentFrame].points;
            renderPoints[j * 3] = points(j, 0);
            renderPoints[j * 3 + 1] = points(j, 1);
            renderPoints[j * 3 + 2] = points(j, 2);
        }
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, renderPoints.size() * sizeof(float), renderPoints.data(), GL_DYNAMIC_DRAW);
        checkGLError("glBufferData");

        // Render
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glEnable(GL_DEPTH_TEST);
        glUseProgram(shaderProgram);

        // Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        // Eigen::Matrix4f mvp = projection * view * model;
        // GLint mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
        // glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.data());
    // Use camera view matrix
        Eigen::Matrix4f view = camera.getViewMatrix();
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f mvp = projection * view * model;
        GLint mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
        glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.data());

        glBindVertexArray(vao);
        glDrawArrays(GL_POINTS, 0, 98);
        checkGLError("glDrawArrays");

        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    glDeleteVertexArrays(1, &vao);
    glDeleteBuffers(1, &vbo);
    glDeleteProgram(shaderProgram);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}