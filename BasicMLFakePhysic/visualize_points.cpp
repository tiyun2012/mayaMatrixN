// GLAD: OpenGL function loader for modern OpenGL (3.3 core profile).
#include <glad/glad.h>

// GLFW: Cross-platform library for creating windows, handling OpenGL contexts, and managing input.
#include <GLFW/glfw3.h>

// ImGui: Immediate-mode GUI library for creating user interfaces.
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// Eigen: High-performance C++ library for linear algebra and matrix operations.
#include <Eigen/Dense>

// Standard libraries
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <iostream>

struct Camera {
    float theta = 0.0f;    // Azimuthal angle (radians)
    float phi = 0.0f;      // Polar angle (radians)
    float radius = 5.0f;   // Distance from origin
    Eigen::Vector3f target = Eigen::Vector3f::Zero(); // Look-at point
    Eigen::Vector3f position;
    bool isRotating = false;
    bool isPanning = false;
    double lastX = 0.0;
    double lastY = 0.0;
    // ey

    // Compute view matrix
    Eigen::Matrix4f getViewMatrix()  {
        Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
        Eigen::Vector3f eye;
        position.x() = target.x() + radius * sinf(phi) * cosf(theta);
        position.y() = target.y() + radius * cosf(phi);
        position.z() = target.z() + radius * sinf(phi) * sinf(theta);
        
        // Look-at matrix
        Eigen::Vector3f forward = (target - position).normalized();
        Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f right = forward.cross(up).normalized();
        up = right.cross(forward).normalized();

        Eigen::Matrix4f lookAt = Eigen::Matrix4f::Identity();
        lookAt.block<1,3>(0,0) = right.transpose();
        lookAt.block<1,3>(1,0) = up.transpose();
        lookAt.block<1,3>(2,0) = -forward.transpose();
        lookAt(0,3) = -right.dot(position);
        lookAt(1,3) = -up.dot(position);
        lookAt(2,3) = forward.dot(position);
        return lookAt;
    }
};

struct FrameData {
    Eigen::MatrixXd transform; // 4x4 transform matrix (double for precision)
    Eigen::MatrixXf points;    // 98x3 raw points (float for OpenGL)
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

    inputMean = Eigen::VectorXf::Zero(3);
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

void cursor_position_callback(GLFWwindow* window, double xpos, double ypos) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;

    Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    double dx = xpos - camera->lastX;
    double dy = ypos - camera->lastY;
    camera->lastX = xpos;
    camera->lastY = ypos;

    if (camera->isRotating) {
        camera->theta += static_cast<float>(dx * 0.005);
        camera->phi += static_cast<float>(dy * 0.005);
        if (camera->phi < 0.1f) camera->phi = 0.1f;
        if (camera->phi > 3.04159f) camera->phi = 3.04159f;
    }
    if (camera->isPanning) {
        Eigen::Vector3f eye;
        eye.x() = camera->target.x() + camera->radius * sinf(camera->phi) * cosf(camera->theta);
        eye.y() = camera->target.y() + camera->radius * cosf(camera->phi);
        eye.z() = camera->target.z() + camera->radius * sinf(camera->phi) * sinf(camera->theta);
        Eigen::Vector3f forward = (camera->target - eye).normalized();
        Eigen::Vector3f up(0.0f, 1.0f, 0.0f);
        Eigen::Vector3f right = forward.cross(up).normalized();
        up = right.cross(forward).normalized();

        float panSpeed = 0.005f * camera->radius;
        camera->target += (-right * static_cast<float>(dx) + up * static_cast<float>(dy)) * panSpeed;
    }
}

void mouse_button_callback(GLFWwindow* window, int button, int action, int mods) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        std::cout << "Control panel context" << std::endl;
        return;
    }
    Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        if (action == GLFW_PRESS) {
            camera->isRotating = true;
            glfwGetCursorPos(window, &camera->lastX, &camera->lastY);
            std::cout << "Viewport left click context" << std::endl;
        } else if (action == GLFW_RELEASE) {
            camera->isRotating = false;
        }
    } else if (button == GLFW_MOUSE_BUTTON_RIGHT) {
        if (action == GLFW_PRESS) {
            camera->isPanning = true;
            glfwGetCursorPos(window, &camera->lastX, &camera->lastY);
            std::cout << "Viewport right click context" << std::endl;
        } else if (action == GLFW_RELEASE) {
            camera->isPanning = false;
        }
    }
}

void scroll_callback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) return;
    Camera* camera = static_cast<Camera*>(glfwGetWindowUserPointer(window));
    camera->radius -= static_cast<float>(yoffset) * 0.5f;
    if (camera->radius < 1.0f) camera->radius = 1.0f;
    if (camera->radius > 50.0f) camera->radius = 50.0f;
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

    // Set input callbacks
    Camera camera;
    glfwSetWindowUserPointer(window, &camera);
    glfwSetMouseButtonCallback(window, mouse_button_callback);
    glfwSetCursorPosCallback(window, cursor_position_callback);
    glfwSetScrollCallback(window, scroll_callback);

    // Initialize GLAD
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
    // Add these global flags at the beginning of main()
    bool show_control_panel = true;
    bool show_debug_info = false;
    bool show_about = false;

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

    // Preload all frames into CPU memory
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
        uniform vec4 inColor;
        void main() {
            FragColor = inColor;
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

    // Projection matrix
    Eigen::Matrix4f projection = Eigen::Matrix4f::Identity();
    float aspect = 1280.0f / 720.0f;
    projection(0, 0) = 1.0f / (aspect * tanf(45.0f * 3.1415926535f / 360.0f));
    projection(1, 1) = 1.0f / tanf(45.0f * 3.1415926535f / 360.0f);
    projection(2, 2) = -(1000.0f + 0.1f) / (1000.0f - 0.1f);
    projection(2, 3) = -2.0f * 1000.0f * 0.1f / (1000.0f - 0.1f);
    projection(3, 2) = -1.0f;
    projection(3, 3) = 0.0f;

    // Grid setup
    std::vector<float> gridVertices;
    int numSteps = 21;
    float gridSize = 10.0f;
    for (int i = 0; i < numSteps; i++) {
        float coord = -gridSize + i * (2 * gridSize) / (numSteps - 1);
        gridVertices.push_back(coord); gridVertices.push_back(0.0f); gridVertices.push_back(-gridSize);
        gridVertices.push_back(coord); gridVertices.push_back(0.0f); gridVertices.push_back(gridSize);
        gridVertices.push_back(-gridSize); gridVertices.push_back(0.0f); gridVertices.push_back(coord);
        gridVertices.push_back(gridSize); gridVertices.push_back(0.0f); gridVertices.push_back(coord);
    }

    GLuint gridVAO, gridVBO;
    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, gridVertices.size() * sizeof(float), gridVertices.data(), GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindVertexArray(0);

    int currentFrame = 0;
    bool showNormalized = false;

    while (!glfwWindowShouldClose(window)) {
        glfwPollEvents();

        // ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        // 1 /Main Window
        // 1. Main Menu Bar
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Reload Data")) {
                // We'll implement this later
                std::cout << "Reload Data clicked" << std::endl;
            }
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Control Panel", NULL, &show_control_panel);
            ImGui::MenuItem("Debug Info", NULL, &show_debug_info);
            ImGui::EndMenu();
        }
        if (ImGui::BeginMenu("Help")) {
            if (ImGui::MenuItem("About")) {
                show_about = true;
            }
            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }
        //2Control Panel
        if(show_control_panel) {
            ImGui::Begin("Control Panel", &show_control_panel);
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
        }
        // 3. Debug Info Window
    if (show_debug_info) {
        ImGui::Begin("Debug Info", &show_debug_info);
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 
                    1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)", 
                    camera.position.x(), camera.position.y(), camera.position.z());
        ImGui::End();
    }

    // 4. About Window
    if (show_about) 
    {
        ImGui::Begin("About Point Cloud Visualizer", &show_about);
        ImGui::Text("Point Cloud Visualizer v0.1");
        ImGui::Separator();
        ImGui::Text("Developed by Your Name");
        ImGui::Text("Using:");
        ImGui::BulletText("GLFW %s", glfwGetVersionString());
        ImGui::BulletText("OpenGL %s", glGetString(GL_VERSION));
        ImGui::BulletText("Dear ImGui %s", IMGUI_VERSION);
        if (ImGui::Button("Close")) show_about = false;
        ImGui::End();
    }

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

        // Use camera view matrix
        Eigen::Matrix4f view = camera.getViewMatrix();
        Eigen::Matrix4f model = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f mvp = projection * view * model;
        GLint mvpLoc = glGetUniformLocation(shaderProgram, "mvp");
        glUniformMatrix4fv(mvpLoc, 1, GL_FALSE, mvp.data());

        // Draw grid plane in gray
        GLint colorLoc = glGetUniformLocation(shaderProgram, "inColor");
        glUniform4f(colorLoc, 0.5f, 0.5f, 0.5f, 1.0f);
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(gridVertices.size() / 3));

        // Draw point cloud in yellow
        glUniform4f(colorLoc, 1.0f, 1.0f, 0.0f, 1.0f);
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
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteProgram(shaderProgram);
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();
    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}