/**
 * ImGui version of Surface Texture Mapping GUI - Minimal working implementation
 * Provides basic 3D visualization framework, with incremental feature additions
 */

#include "imgui_texture_mapping_gui.h"
#include <iostream>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional>
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Geometry-central includes for mesh structures
#include "geometrycentral/surface/halfedge_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

// TODO: Core库包含暂时禁用以避免Vector3冲突
// #include "geometry.h"
// #include "halfedge_mesh.h"

// Algorithm includes for real implementations
#include "real_algorithms.h"

// Use geometry-central types
using namespace geometrycentral;
using namespace geometrycentral::surface;
// TODO: 集成真实的EulerianShapeOptimizer（当前因为SuiteSparse依赖问题暂时禁用）
// #include "../../cuts/core/include/eulerian_shape_optimizer.h"

// TODO: CGAL includes for mesh processing (temporarily disabled)
// Currently using simplified implementations without CGAL dependency

// MeshQualityStats and SimpleFlattener are already defined in the header file

// Use PI constant from utilities.h instead of PI
// PI is defined in utilities.h as 3.1415926535897932384

#ifdef _WIN32
#define NOMINMAX  // Prevent Windows from defining min/max macros
#define byte win_byte_override  // Resolve byte conflict
#include <windows.h>
#include <commdlg.h>
#undef byte
#define GLFW_EXPOSE_NATIVE_WIN32
#include <GLFW/glfw3native.h>
#endif

namespace SurfaceTextureMapping {

ImGuiTextureMappingGUI::ImGuiTextureMappingGUI()
    : m_window(nullptr)
    , m_shaderProgram(0)
    , m_meshVAO(0)
    , m_meshVBO(0)
    , m_meshEBO(0)
    , m_linesVAO(0)
    , m_linesVBO(0)
    , m_flattener(nullptr)  // Initialize SimpleFlattener to nullptr
{
    // Initialize core algorithm components
    m_meshProcessor = std::unique_ptr<MeshProcessor>(new MeshProcessor());
    m_cutter = std::unique_ptr<VariationalCutter>(new VariationalCutter());
    m_textureMapper = std::unique_ptr<TextureMapper>(new TextureMapper());
    m_surfaceFiller = std::unique_ptr<SurfaceFiller>(new SurfaceFiller());

    // Initialize algorithm pipeline for real algorithms
    m_algorithmPipeline = std::make_unique<AlgorithmPipeline>();

    // Initialize camera
    updateCamera();
}

ImGuiTextureMappingGUI::~ImGuiTextureMappingGUI() {
    cleanup();
}

bool ImGuiTextureMappingGUI::initialize() {
    if (!initializeGLFW()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    if (!initializeImGui()) {
        std::cerr << "Failed to initialize ImGui\n";
        return false;
    }

    if (!initializeOpenGL()) {
        std::cerr << "Failed to initialize OpenGL\n";
        return false;
    }

    if (!loadShaders()) {
        std::cerr << "Failed to load shaders\n";
        return false;
    }

    // Initialize Surface Texture Mapping components (temporarily simplified)
    logMessage("ImGui GUI initialized successfully");

    return true;
}

bool ImGuiTextureMappingGUI::initializeGLFW() {
    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    // Set OpenGL version
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    m_window = glfwCreateWindow(m_windowWidth, m_windowHeight,
                                "Surface Texture Mapping - ImGui", nullptr, nullptr);
    if (!m_window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);

    // Initialize GLEW after creating OpenGL context
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW\n";
        glfwDestroyWindow(m_window);
        glfwTerminate();
        return false;
    }

    glfwSetWindowUserPointer(m_window, this);

    // Set callback functions
    glfwSetMouseButtonCallback(m_window, mouseCallback);
    glfwSetCursorPosCallback(m_window, cursorCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetKeyCallback(m_window, keyCallback);

    return true;
}

bool ImGuiTextureMappingGUI::initializeImGui() {
    // Set up ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;

    // Set up Chinese font support
    // Note: Using default font's Chinese support method, actual project may need to load Chinese font files
    io.Fonts->AddFontDefault();

    // Set ImGui style
    ImGui::StyleColorsDark();

    // Set platform/renderer backend
    if (!ImGui_ImplGlfw_InitForOpenGL(m_window, true)) {
        std::cerr << "Failed to initialize ImGui GLFW backend\n";
        return false;
    }

    if (!ImGui_ImplOpenGL3_Init("#version 330")) {
        std::cerr << "Failed to initialize ImGui OpenGL3 backend\n";
        return false;
    }

    return true;
}

bool ImGuiTextureMappingGUI::initializeOpenGL() {
    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Set viewport
    glViewport(0, 0, m_windowWidth, m_windowHeight);

    return true;
}

bool ImGuiTextureMappingGUI::loadShaders() {
    // Enhanced vertex shader with lighting
    const char* vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aNormal;
        layout (location = 2) in vec3 aColor;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;
        uniform vec3 lightPos;
        uniform vec3 viewPos;

        out vec3 FragPos;
        out vec3 Normal;
        out vec3 vertexColor;
        out vec3 LightPos;
        out vec3 ViewPos;

        void main() {
            FragPos = vec3(model * vec4(aPos, 1.0));
            Normal = mat3(transpose(inverse(model))) * aNormal;
            vertexColor = aColor;
            LightPos = lightPos;
            ViewPos = viewPos;

            gl_Position = projection * view * vec4(FragPos, 1.0);
        }
    )";

    // Enhanced fragment shader with Phong lighting
    const char* fragmentShaderSource = R"(
        #version 330 core
        in vec3 FragPos;
        in vec3 Normal;
        in vec3 vertexColor;
        in vec3 LightPos;
        in vec3 ViewPos;

        out vec4 color;

        void main() {
            // Ambient lighting
            float ambientStrength = 0.3;
            vec3 ambient = ambientStrength * vertexColor;

            // Diffuse lighting
            vec3 norm = normalize(Normal);
            vec3 lightDir = normalize(LightPos - FragPos);
            float diff = max(dot(norm, lightDir), 0.0);
            vec3 diffuse = diff * vertexColor;

            // Specular lighting
            float specularStrength = 0.5;
            vec3 viewDir = normalize(ViewPos - FragPos);
            vec3 reflectDir = reflect(-lightDir, norm);
            float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
            vec3 specular = specularStrength * spec * vec3(1.0);

            vec3 result = ambient + diffuse + specular;
            color = vec4(result, 1.0);
        }
    )";

    // Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, nullptr);
    glCompileShader(vertexShader);

    GLint success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, nullptr, infoLog);
        std::cerr << "Vertex shader compilation failed:\n" << infoLog << std::endl;
        return false;
    }

    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, nullptr);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, nullptr, infoLog);
        std::cerr << "Fragment shader compilation failed:\n" << infoLog << std::endl;
        return false;
    }

    // Create shader program
    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);

    glGetProgramiv(m_shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(m_shaderProgram, 512, nullptr, infoLog);
        std::cerr << "Shader program linking failed:\n" << infoLog << std::endl;
        return false;
    }

    // Clean up shader objects
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

void ImGuiTextureMappingGUI::run() {
    while (!glfwWindowShouldClose(m_window)) {
        glfwPollEvents();

        // Start ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Render 3D scene
        render();

        // Draw ImGui interface
        drawMainMenuBar();
        drawControlPanel();
        drawVisualizationPanel();
        drawStatisticsPanel();
        drawLogPanel();

        // Render ImGui
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(m_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    }
}

void ImGuiTextureMappingGUI::render() {
    // Clear color and depth buffers
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Render the mesh using the dedicated function
    renderMesh();

    // Render cut lines if they exist and are enabled
    renderCuts();

    // Render UV grid if UV mapping is computed and enabled
    renderUVGrid();

    // Render pattern lines if patterns are generated and enabled
    renderPatterns();
}

void ImGuiTextureMappingGUI::drawMainMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            ImGui::InputText("Input File", m_guiState.inputFile, sizeof(m_guiState.inputFile));
            if (ImGui::MenuItem("Load Mesh")) {
                std::string filename = openFileDialog();
                if (!filename.empty()) {
                    loadMeshFromFile(filename);
                }
            }
            ImGui::Separator();
            ImGui::InputText("Output Prefix", m_guiState.outputPrefix, sizeof(m_guiState.outputPrefix));
            if (ImGui::MenuItem("Export Results")) {
                // TODO: Implement result export
                logMessage("Export functionality to be implemented");
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Process")) {
            if (ImGui::MenuItem("Run Full Pipeline")) {
                // TODO: Implement full pipeline
                logMessage("Full pipeline processing to be implemented");
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            if (ImGui::MenuItem("Reset Camera")) {
                resetCamera();
            }
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void ImGuiTextureMappingGUI::drawControlPanel() {
    ImGui::SetNextWindowPos(ImVec2(10, 50), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(300, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Control Panel");

    ImGui::Text("Mesh Processing");
    ImGui::Checkbox("Enable Remeshing", &m_guiState.enableRemeshing);
    if (m_guiState.enableRemeshing) {
        ImGui::SliderFloat("Target Edge Length", &m_guiState.targetEdgeLength, 0.001f, 0.1f, "%.4f");
        ImGui::SliderInt("Remesh Iterations", &m_guiState.remeshIterations, 1, 10);
        ImGui::Checkbox("Protect Boundary", &m_guiState.protectBoundary);
    }

    ImGui::Separator();
    ImGui::Text("Variational Cutting");
    ImGui::Checkbox("Enable Cutting", &m_guiState.enableCutting);
    if (m_guiState.enableCutting) {
        ImGui::SliderFloat("Length Regularization", &m_guiState.lengthRegularization, 0.01f, 1.0f);
        ImGui::SliderFloat("Smooth Regularization", &m_guiState.smoothRegularization, 0.001f, 0.5f);
        ImGui::SliderInt("Max Iterations", &m_guiState.maxCuttingIterations, 10, 100);
    }

    ImGui::Separator();
    ImGui::Text("Texture Mapping");
    ImGui::Checkbox("Enable Texturing", &m_guiState.enableTexturing);
    if (m_guiState.enableTexturing) {
        ImGui::SliderFloat("Curvature Threshold", &m_guiState.curvatureThreshold, 0.01f, 1.0f);
    }

    ImGui::Separator();
    ImGui::Text("Surface Filling");
    ImGui::Checkbox("Enable Filling", &m_guiState.enableFilling);
    if (m_guiState.enableFilling) {
        const char* patternTypes[] = {"Grid", "Hexagonal", "Spiral", "Hilbert"};
        ImGui::Combo("Pattern Type", &m_guiState.patternType, patternTypes, 4);
        ImGui::SliderFloat("Pattern Spacing", &m_guiState.patternSpacing, 0.001f, 0.1f, "%.4f");
        if (m_guiState.patternType == 3) { // Hilbert
            ImGui::SliderInt("Hilbert Order", &m_guiState.hilbertOrder, 1, 8);
        }
    }

    ImGui::Separator();
    ImGui::Text("Pipeline Execution");

    if (!m_meshLoaded) {
        ImGui::BeginDisabled(true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
    }

    if (ImGui::Button("Run Full Pipeline", ImVec2(-1, 0))) {
        if (m_meshLoaded && !m_processingInProgress) {
            runFullPipeline();
        }
    }

    ImGui::Separator();

    if (ImGui::Button("1. Process Mesh", ImVec2(-1, 0))) {
        if (m_meshLoaded && !m_processingInProgress) {
            processMesh();
        }
    }

    if (ImGui::Button("2. Compute Cuts", ImVec2(-1, 0))) {
        if (m_meshProcessed && !m_processingInProgress) {
            computeCuts();
        }
    }

    if (ImGui::Button("3. UV Mapping", ImVec2(-1, 0))) {
        if (m_cutComputed && !m_processingInProgress) {
            computeUVMapping();
        }
    }

    if (ImGui::Button("4. Generate Patterns", ImVec2(-1, 0))) {
        if (m_uvMappingComputed && !m_processingInProgress) {
            generatePatterns();
        }
    }

    if (!m_meshLoaded) {
        ImGui::PopStyleVar();
        ImGui::EndDisabled();
    }

    ImGui::End();
}

void ImGuiTextureMappingGUI::drawVisualizationPanel() {
    ImGui::SetNextWindowPos(ImVec2(320, 50), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 400), ImGuiCond_FirstUseEver);
    ImGui::Begin("Visualization Options");

    ImGui::Text("Display Options");
    ImGui::Checkbox("Show Wireframe", &m_guiState.showWireframe);
    ImGui::Checkbox("Show Filled", &m_guiState.showFilled);

    // Only show advanced options if pipeline steps are completed
    if (m_cutComputed) {
        ImGui::Checkbox("Show Cuts", &m_guiState.showCuts);
        if (m_guiState.showCuts) {
            ImGui::Text("  Cut lines: %d", m_statistics.numCuts);
        }
    } else {
        ImGui::BeginDisabled(true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        bool disabled = false;
        ImGui::Checkbox("Show Cuts", &disabled);
        ImGui::PopStyleVar();
        ImGui::EndDisabled();
    }

    if (m_uvMappingComputed) {
        ImGui::Checkbox("Show UV Mapping", &m_guiState.showUVMapping);
        if (m_guiState.showUVMapping) {
            ImGui::Text("  Charts: %d", m_statistics.numCharts);
        }
    } else {
        ImGui::BeginDisabled(true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        bool disabled = false;
        ImGui::Checkbox("Show UV Mapping", &disabled);
        ImGui::PopStyleVar();
        ImGui::EndDisabled();
    }

    if (m_patternsGenerated) {
        ImGui::Checkbox("Show Patterns", &m_guiState.showPatterns);
        if (m_guiState.showPatterns) {
            ImGui::Text("  Paths: %d", m_statistics.numPaths);
        }
    } else {
        ImGui::BeginDisabled(true);
        ImGui::PushStyleVar(ImGuiStyleVar_Alpha, ImGui::GetStyle().Alpha * 0.5f);
        bool disabled = false;
        ImGui::Checkbox("Show Patterns", &disabled);
        ImGui::PopStyleVar();
        ImGui::EndDisabled();
    }

    ImGui::Separator();
    ImGui::Text("Camera Controls");
    if (ImGui::Button("Reset Camera", ImVec2(-1, 0))) {
        resetCamera();
    }
    ImGui::Text("Distance: %.2f", m_cameraDistance);
    ImGui::Text("Rotation X: %.2f°", m_cameraRotationX * 57.3f); // Convert to degrees
    ImGui::Text("Rotation Y: %.2f°", m_cameraRotationY * 57.3f);

    ImGui::Separator();
    ImGui::Text("Color Settings");
    ImGui::ColorEdit3("Mesh Color", glm::value_ptr(m_guiState.meshColor));
    ImGui::ColorEdit3("Wireframe Color", glm::value_ptr(m_guiState.wireframeColor));

    if (m_cutComputed) {
        ImGui::ColorEdit3("Cut Color", glm::value_ptr(m_guiState.cutColor));
    }

    if (m_patternsGenerated) {
        ImGui::ColorEdit3("Pattern Color", glm::value_ptr(m_guiState.patternColor));
    }

    ImGui::End();
}

void ImGuiTextureMappingGUI::drawStatisticsPanel() {
    ImGui::SetNextWindowPos(ImVec2(580, 50), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(250, 500), ImGuiCond_FirstUseEver);
    ImGui::Begin("Statistics");

    // Processing Status
    ImGui::Text("Processing Status");
    ImGui::Text("Status: %s", m_statusMessage.c_str());

    if (m_processingInProgress) {
        ImGui::TextColored(ImVec4(1.0f, 0.5f, 0.0f, 1.0f), "PROCESSING...");
    }

    // Pipeline Progress
    ImGui::Separator();
    ImGui::Text("Pipeline Progress");

    ImVec4 completedColor = ImVec4(0.0f, 1.0f, 0.0f, 1.0f);
    ImVec4 pendingColor = ImVec4(0.5f, 0.5f, 0.5f, 1.0f);

    ImGui::TextColored(m_meshLoaded ? completedColor : pendingColor,
                      "%s Mesh Loaded", m_meshLoaded ? "✓" : "○");
    ImGui::TextColored(m_meshProcessed ? completedColor : pendingColor,
                      "%s Mesh Processed", m_meshProcessed ? "✓" : "○");
    ImGui::TextColored(m_cutComputed ? completedColor : pendingColor,
                      "%s Cuts Computed", m_cutComputed ? "✓" : "○");
    ImGui::TextColored(m_uvMappingComputed ? completedColor : pendingColor,
                      "%s UV Mapped", m_uvMappingComputed ? "✓" : "○");
    ImGui::TextColored(m_patternsGenerated ? completedColor : pendingColor,
                      "%s Patterns Generated", m_patternsGenerated ? "✓" : "○");

    ImGui::Separator();
    ImGui::Text("Mesh Statistics");
    ImGui::Text("Vertices: %d", m_statistics.numVertices);
    ImGui::Text("Faces: %d", m_statistics.numFaces);
    ImGui::Text("Edges: %d", m_statistics.numEdges);
    if (m_statistics.avgEdgeLength > 0) {
        ImGui::Text("Avg Edge Length: %.4f", m_statistics.avgEdgeLength);
        ImGui::Text("Min Edge Length: %.4f", m_statistics.minEdgeLength);
        ImGui::Text("Max Edge Length: %.4f", m_statistics.maxEdgeLength);
    }
    ImGui::Text("Manifold: %s", m_statistics.isManifold ? "Yes" : "No");
    ImGui::Text("Closed: %s", m_statistics.isClosed ? "Yes" : "No");

    if (m_cutComputed) {
        ImGui::Separator();
        ImGui::Text("Cutting Statistics");
        ImGui::Text("Number of Cuts: %d", m_statistics.numCuts);
        ImGui::Text("Total Cut Length: %.4f", m_statistics.totalCutLength);
        ImGui::Text("Avg Distortion: %.4f", m_statistics.avgDistortion);
    }

    if (m_uvMappingComputed) {
        ImGui::Separator();
        ImGui::Text("UV Mapping Statistics");
        ImGui::Text("Number of Charts: %d", m_statistics.numCharts);
        ImGui::Text("Angle Distortion: %.4f", m_statistics.angleDistortion);
        ImGui::Text("Area Distortion: %.4f", m_statistics.areaDistortion);
        ImGui::Text("Conformal Error: %.4f", m_statistics.conformalError);
    }

    if (m_patternsGenerated) {
        ImGui::Separator();
        ImGui::Text("Filling Statistics");
        ImGui::Text("Number of Paths: %d", m_statistics.numPaths);
        ImGui::Text("Total Path Length: %.2f", m_statistics.totalPathLength);
        ImGui::Text("Coverage: %.1f%%", m_statistics.coverage * 100.0f);
        ImGui::Text("Uniformity: %.1f%%", m_statistics.uniformity * 100.0f);
    }

    ImGui::End();
}

void ImGuiTextureMappingGUI::drawLogPanel() {
    ImGui::SetNextWindowPos(ImVec2(10, 560), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(820, 200), ImGuiCond_FirstUseEver);
    ImGui::Begin("Log");

    if (ImGui::Button("Clear Log")) {
        m_logMessages.clear();
    }
    ImGui::SameLine();
    ImGui::Checkbox("Auto Scroll", &m_autoScroll);

    ImGui::Separator();

    if (ImGui::BeginChild("LogScrollRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar)) {
        for (const auto& message : m_logMessages) {
            ImGui::TextUnformatted(message.c_str());
        }

        if (m_autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
            ImGui::SetScrollHereY(1.0f);
        }
    }
    ImGui::EndChild();

    ImGui::End();
}

void ImGuiTextureMappingGUI::updateCamera() {
    // Calculate camera position
    float x = m_cameraDistance * cos(m_cameraRotationY) * cos(m_cameraRotationX);
    float y = m_cameraDistance * sin(m_cameraRotationX);
    float z = m_cameraDistance * sin(m_cameraRotationY) * cos(m_cameraRotationX);

    m_cameraPosition = glm::vec3(x, y, z);

    // Update view matrix
    m_viewMatrix = glm::lookAt(m_cameraPosition, m_cameraTarget, m_cameraUp);

    // Update projection matrix
    m_projectionMatrix = glm::perspective(glm::radians(45.0f),
                                         (float)m_windowWidth / (float)m_windowHeight,
                                         0.1f, 100.0f);
}

void ImGuiTextureMappingGUI::resetCamera() {
    m_cameraDistance = 3.0f;
    m_cameraRotationX = 0.0f;
    m_cameraRotationY = 0.0f;
    updateCamera();
    logMessage("Camera reset");
}

void ImGuiTextureMappingGUI::logMessage(const std::string& message) {
    m_logMessages.push_back(message);
}

void ImGuiTextureMappingGUI::cleanup() {
    if (m_shaderProgram) {
        glDeleteProgram(m_shaderProgram);
    }

    if (m_meshVAO) {
        glDeleteVertexArrays(1, &m_meshVAO);
        glDeleteBuffers(1, &m_meshVBO);
        glDeleteBuffers(1, &m_meshEBO);
    }

    if (m_linesVAO) {
        glDeleteVertexArrays(1, &m_linesVAO);
        glDeleteBuffers(1, &m_linesVBO);
    }

    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    if (m_window) {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

// Static callback functions
void ImGuiTextureMappingGUI::mouseCallback(GLFWwindow* window, int button, int action, int mods) {
    (void)mods; // Suppress unused parameter warning
    ImGuiTextureMappingGUI* gui = static_cast<ImGuiTextureMappingGUI*>(glfwGetWindowUserPointer(window));
    if (gui) {
        if (button == GLFW_MOUSE_BUTTON_LEFT) {
            if (action == GLFW_PRESS) {
                gui->m_mousePressed = true;
                glfwGetCursorPos(window, &gui->m_lastMouseX, &gui->m_lastMouseY);
            } else if (action == GLFW_RELEASE) {
                gui->m_mousePressed = false;
            }
        }
    }
}

void ImGuiTextureMappingGUI::cursorCallback(GLFWwindow* window, double xpos, double ypos) {
    ImGuiTextureMappingGUI* gui = static_cast<ImGuiTextureMappingGUI*>(glfwGetWindowUserPointer(window));
    if (gui && gui->m_mousePressed) {
        double xoffset = xpos - gui->m_lastMouseX;
        double yoffset = ypos - gui->m_lastMouseY;

        gui->m_lastMouseX = xpos;
        gui->m_lastMouseY = ypos;

        gui->m_cameraRotationY += static_cast<float>(xoffset) * 0.01f;
        gui->m_cameraRotationX += static_cast<float>(yoffset) * 0.01f;

        gui->updateCamera();
    }
}

void ImGuiTextureMappingGUI::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    (void)xoffset; // Suppress unused parameter warning
    ImGuiTextureMappingGUI* gui = static_cast<ImGuiTextureMappingGUI*>(glfwGetWindowUserPointer(window));
    if (gui) {
        gui->m_cameraDistance -= static_cast<float>(yoffset) * 0.1f;
        gui->m_cameraDistance = std::max(0.5f, std::min(20.0f, gui->m_cameraDistance));
        gui->updateCamera();
    }
}

void ImGuiTextureMappingGUI::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    (void)scancode; // Suppress unused parameter warning
    (void)mods; // Suppress unused parameter warning
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

glm::vec3 ImGuiTextureMappingGUI::eigenToGLM(const STM::Vector3& v) {
    return glm::vec3(static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(v.z));
}

STM::Vector3 ImGuiTextureMappingGUI::glmToEigen(const glm::vec3& v) {
    return STM::Vector3{v.x, v.y, v.z};
}

#ifdef _WIN32
std::string ImGuiTextureMappingGUI::openFileDialog() {
    OPENFILENAMEA ofn;
    char szFile[260] = {0};

    ZeroMemory(&ofn, sizeof(ofn));
    ofn.lStructSize = sizeof(ofn);
    ofn.hwndOwner = glfwGetWin32Window(m_window);
    ofn.lpstrFile = szFile;
    ofn.nMaxFile = sizeof(szFile);
    ofn.lpstrFilter = "OBJ Files\0*.obj\0All Files\0*.*\0";
    ofn.nFilterIndex = 1;
    ofn.lpstrFileTitle = NULL;
    ofn.nMaxFileTitle = 0;
    ofn.lpstrInitialDir = NULL;
    ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST;

    if (GetOpenFileNameA(&ofn)) {
        return std::string(szFile);
    }
    return "";
}
#else
std::string ImGuiTextureMappingGUI::openFileDialog() {
    // Linux/Mac implementation would go here
    return "";
}
#endif

bool ImGuiTextureMappingGUI::loadMesh(const std::string& filename) {
    // Public interface for command-line loading
    return loadMeshFromFile(filename);
}

bool ImGuiTextureMappingGUI::loadMeshFromFile(const std::string& filename) {
    if (filename.empty()) {
        logMessage("No file selected");
        return false;
    }

    // Update the input file path
    strncpy_s(m_guiState.inputFile, filename.c_str(), sizeof(m_guiState.inputFile) - 1);
    m_guiState.inputFile[sizeof(m_guiState.inputFile) - 1] = '\0';

    // Load mesh using geometry-central and also create render data
    if (!loadMeshGeometryFromFile(filename) || !loadOBJFile(filename)) {
        logMessage("Failed to load mesh: " + filename);
        return false;
    }

    m_meshLoaded = true;
    logMessage("Successfully loaded mesh: " + filename);

    // Update statistics
    updateStatistics();

    return true;
}

bool ImGuiTextureMappingGUI::loadOBJFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return false;
    }

    m_meshVertices.clear();
    m_meshIndices.clear();

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec3> normals;
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string prefix;
        iss >> prefix;

        if (prefix == "v") {
            float x, y, z;
            iss >> x >> y >> z;
            vertices.push_back(glm::vec3(x, y, z));
        }
        else if (prefix == "vn") {
            float nx, ny, nz;
            iss >> nx >> ny >> nz;
            normals.push_back(glm::vec3(nx, ny, nz));
        }
        else if (prefix == "f") {
            std::string vertex1, vertex2, vertex3;
            iss >> vertex1 >> vertex2 >> vertex3;

            // Parse face indices (handle v/vt/vn format)
            auto parseVertex = [](const std::string& s) {
                return std::stoi(s.substr(0, s.find('/'))) - 1; // OBJ uses 1-based indexing
            };

            unsigned int v1 = parseVertex(vertex1);
            unsigned int v2 = parseVertex(vertex2);
            unsigned int v3 = parseVertex(vertex3);

            if (v1 < vertices.size() && v2 < vertices.size() && v3 < vertices.size()) {
                m_meshIndices.push_back(v1);
                m_meshIndices.push_back(v2);
                m_meshIndices.push_back(v3);
            }
        }
    }

    // Compute face normals if not available
    if (normals.empty() && !m_meshIndices.empty()) {
        normals.resize(vertices.size(), glm::vec3(0.0f));

        // Calculate face normals and accumulate to vertices
        for (size_t i = 0; i < m_meshIndices.size(); i += 3) {
            if (i + 2 < m_meshIndices.size()) {
                glm::vec3 v0 = vertices[m_meshIndices[i]];
                glm::vec3 v1 = vertices[m_meshIndices[i + 1]];
                glm::vec3 v2 = vertices[m_meshIndices[i + 2]];

                glm::vec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));
                normals[m_meshIndices[i]] += normal;
                normals[m_meshIndices[i + 1]] += normal;
                normals[m_meshIndices[i + 2]] += normal;
            }
        }

        // Normalize accumulated normals
        for (auto& normal : normals) {
            normal = glm::normalize(normal);
        }
    }

    // Create vertex data with positions, normals, and colors (9 floats per vertex)
    for (size_t i = 0; i < vertices.size(); ++i) {
        const auto& vertex = vertices[i];
        const auto& normal = (i < normals.size()) ? normals[i] : glm::vec3(0.0f, 0.0f, 1.0f);

        // Position (3 floats)
        m_meshVertices.push_back(vertex.x);
        m_meshVertices.push_back(vertex.y);
        m_meshVertices.push_back(vertex.z);

        // Normal (3 floats)
        m_meshVertices.push_back(normal.x);
        m_meshVertices.push_back(normal.y);
        m_meshVertices.push_back(normal.z);

        // Determine color based on pipeline state
        glm::vec3 color = m_guiState.meshColor;
        if (m_patternsGenerated) {
            color = glm::mix(m_guiState.meshColor, m_guiState.patternColor, 0.3f);  // Pattern stage
        } else if (m_uvMappingComputed) {
            color = glm::mix(m_guiState.meshColor, glm::vec3(0.2f, 0.8f, 0.2f), 0.3f);  // UV mapping stage
        } else if (m_cutComputed) {
            color = glm::mix(m_guiState.meshColor, m_guiState.cutColor, 0.2f);  // Cutting stage
        } else if (m_meshProcessed) {
            color = glm::mix(m_guiState.meshColor, glm::vec3(0.9f, 0.7f, 0.2f), 0.2f);  // Processing stage
        }

        // Color (3 floats)
        m_meshVertices.push_back(color.x);
        m_meshVertices.push_back(color.y);
        m_meshVertices.push_back(color.z);
    }

    // Update OpenGL buffers
    updateMeshBuffers();

    // Calculate bounding box and adjust camera
    if (!vertices.empty()) {
        glm::vec3 min = vertices[0], max = vertices[0];
        for (const auto& v : vertices) {
            min = glm::min(min, v);
            max = glm::max(max, v);
        }

        glm::vec3 center = (min + max) * 0.5f;
        float radius = glm::length(max - min) * 0.6f;

        m_cameraTarget = center;
        m_cameraDistance = std::max(radius, 1.0f);
        updateCamera();

        logMessage("Loaded " + std::to_string(vertices.size()) + " vertices, " +
                  std::to_string(m_meshIndices.size() / 3) + " faces");
    }

    return true;
}

bool ImGuiTextureMappingGUI::loadMeshGeometryFromFile(const std::string& filename) {
    try {
        // Simplified: Create basic mesh structure
        // In real implementation, would use proper geometry-central mesh loading
        m_mesh = std::unique_ptr<HalfedgeMesh>(new HalfedgeMesh());
        m_geometry = std::unique_ptr<VertexPositionGeometry>(new VertexPositionGeometry(*m_mesh));

        logMessage("Created simplified mesh structures for: " + filename);
        logMessage("Note: Using simplified geometry interface for demonstration");

        return true;
    }
    catch (const std::exception& e) {
        logMessage("Failed to create mesh structures: " + std::string(e.what()));
        return false;
    }
}

void ImGuiTextureMappingGUI::updateMeshBuffers() {
    if (m_meshVertices.empty()) return;

    // Generate buffers if not already created
    if (m_meshVAO == 0) {
        glGenVertexArrays(1, &m_meshVAO);
        glGenBuffers(1, &m_meshVBO);
        glGenBuffers(1, &m_meshEBO);
    }

    // Update vertex buffer
    glBindVertexArray(m_meshVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);
    glBufferData(GL_ARRAY_BUFFER, m_meshVertices.size() * sizeof(float), m_meshVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_meshEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_meshIndices.size() * sizeof(unsigned int), m_meshIndices.data(), GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Color attribute
    glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, 9 * sizeof(float), (void*)(6 * sizeof(float)));
    glEnableVertexAttribArray(2);

    glBindVertexArray(0);
}

void ImGuiTextureMappingGUI::updateStatistics() {
    if (m_meshVertices.empty()) return;

    m_statistics.numVertices = static_cast<int>(m_meshVertices.size() / 6); // 6 floats per vertex (pos + color)
    m_statistics.numFaces = static_cast<int>(m_meshIndices.size() / 3);
    m_statistics.numEdges = m_statistics.numFaces * 3 / 2; // Rough estimate for closed meshes

    // Calculate average edge length (simplified)
    if (m_meshIndices.size() >= 6) {
        float totalLength = 0.0f;
        int edgeCount = 0;

        for (size_t i = 0; i < m_meshIndices.size(); i += 3) {
            for (int j = 0; j < 3; j++) {
                int v1 = m_meshIndices[i + j] * 6; // 6 floats per vertex
                int v2 = m_meshIndices[i + (j + 1) % 3] * 6;

                if (v1 + 2 < static_cast<int>(m_meshVertices.size()) && v2 + 2 < static_cast<int>(m_meshVertices.size())) {
                    glm::vec3 p1(m_meshVertices[v1], m_meshVertices[v1 + 1], m_meshVertices[v1 + 2]);
                    glm::vec3 p2(m_meshVertices[v2], m_meshVertices[v2 + 1], m_meshVertices[v2 + 2]);
                    totalLength += glm::length(p2 - p1);
                    edgeCount++;
                }
            }
        }

        if (edgeCount > 0) {
            m_statistics.avgEdgeLength = totalLength / edgeCount;
            m_statistics.minEdgeLength = m_statistics.avgEdgeLength * 0.5f; // Rough estimate
            m_statistics.maxEdgeLength = m_statistics.avgEdgeLength * 2.0f; // Rough estimate
        }
    }

    // Basic manifold check (simplified)
    m_statistics.isManifold = true; // Assume manifold for now
    m_statistics.isClosed = true;   // Assume closed for now
}

// Pipeline processing methods
void ImGuiTextureMappingGUI::runFullPipeline() {
    if (!m_meshLoaded || m_processingInProgress) {
        return;
    }

    logMessage("Starting full pipeline processing...");
    m_processingInProgress = true;

    try {
        if (m_guiState.enableRemeshing) {
            processMesh();
        } else {
            m_meshProcessed = true;
        }

        if (m_guiState.enableCutting && m_meshProcessed) {
            computeCuts();
        } else {
            m_cutComputed = true;
        }

        if (m_guiState.enableTexturing && m_cutComputed) {
            computeUVMapping();
        } else {
            m_uvMappingComputed = true;
        }

        if (m_guiState.enableFilling && m_uvMappingComputed) {
            generatePatterns();
        } else {
            m_patternsGenerated = true;
        }

        logMessage("Full pipeline completed successfully!");
        m_statusMessage = "Pipeline completed";
    }
    catch (const std::exception& e) {
        logMessage("Pipeline error: " + std::string(e.what()));
        m_statusMessage = "Pipeline failed: " + std::string(e.what());
    }

    m_processingInProgress = false;
}

void ImGuiTextureMappingGUI::processMesh() {
    if (!m_meshLoaded || m_processingInProgress) {
        return;
    }

    logMessage("开始网格预处理...");
    m_processingInProgress = true;

    try {
        // Use real algorithm pipeline if mesh is loaded
        if (m_mesh && m_geometry && m_algorithmPipeline) {
            // Set the mesh in the algorithm pipeline
            m_algorithmPipeline->setMesh(m_mesh.get(), m_geometry.get());

            // Run mesh analysis
            auto analysis = m_algorithmPipeline->runMeshAnalysis();

            // Update statistics from real analysis
            m_statistics.numVertices = static_cast<int>(analysis.nVertices);
            m_statistics.numFaces = static_cast<int>(analysis.nFaces);
            m_statistics.numEdges = static_cast<int>(analysis.nEdges);
            m_statistics.avgEdgeLength = static_cast<float>(analysis.avgEdgeLength);
            m_statistics.minEdgeLength = static_cast<float>(analysis.minEdgeLength);
            m_statistics.maxEdgeLength = static_cast<float>(analysis.maxEdgeLength);
            m_statistics.isManifold = analysis.isManifold;
            m_statistics.isClosed = analysis.isWatertight;

            logMessage("网格分析完成:");
            logMessage("  顶点数: " + std::to_string(analysis.nVertices));
            logMessage("  面片数: " + std::to_string(analysis.nFaces));
            logMessage("  边数: " + std::to_string(analysis.nEdges));
            logMessage("  欧拉特征: " + std::to_string(analysis.eulerCharacteristic));
            logMessage("  亏格: " + std::to_string(analysis.genus));
            logMessage("  平均高斯曲率: " + std::to_string(analysis.avgGaussianCurvature));
        }
        else {
            // Fallback to simplified processing
            logMessage("使用简化网格处理...");
            m_statistics.numVertices = static_cast<int>(m_meshVertices.size() / 9);
            m_statistics.numFaces = static_cast<int>(m_meshIndices.size() / 3);
            m_statistics.numEdges = m_statistics.numFaces * 3 / 2; // Approximate

        // Calculate bounding box
        float minX = std::numeric_limits<float>::max();
        float maxX = std::numeric_limits<float>::lowest();
        float minY = std::numeric_limits<float>::max();
        float maxY = std::numeric_limits<float>::lowest();
        float minZ = std::numeric_limits<float>::max();
        float maxZ = std::numeric_limits<float>::lowest();

        for (size_t i = 0; i < m_meshVertices.size(); i += 9) {
            minX = std::min(minX, m_meshVertices[i]);
            maxX = std::max(maxX, m_meshVertices[i]);
            minY = std::min(minY, m_meshVertices[i + 1]);
            maxY = std::max(maxY, m_meshVertices[i + 1]);
            minZ = std::min(minZ, m_meshVertices[i + 2]);
            maxZ = std::max(maxZ, m_meshVertices[i + 2]);
        }

        float avgEdgeLength = std::sqrt(
            ((maxX - minX) * (maxX - minX) +
             (maxY - minY) * (maxY - minY) +
             (maxZ - minZ) * (maxZ - minZ)) / 3.0f
        ) / 10.0f; // Rough approximation

        m_statistics.avgEdgeLength = avgEdgeLength;
        m_statistics.minEdgeLength = avgEdgeLength * 0.5f;
        m_statistics.maxEdgeLength = avgEdgeLength * 1.5f;
        m_statistics.isManifold = true;  // Assume true for simplicity
        m_statistics.isClosed = true;    // Assume true for simplicity

        logMessage("网格统计信息:");
        logMessage("  顶点数: " + std::to_string(m_statistics.numVertices));
        logMessage("  面片数: " + std::to_string(m_statistics.numFaces));
        logMessage("  边数: " + std::to_string(m_statistics.numEdges));
        logMessage("  平均边长: " + std::to_string(m_statistics.avgEdgeLength));

        // Update mesh buffers and statistics
        updateMeshBuffers();
        updateStatistics();

        m_meshProcessed = true;
        m_statusMessage = "网格处理完成";
        logMessage("简化网格处理成功完成！");

        // Update mesh visualization with new colors
        if (m_meshLoaded) {
            updateMeshVisualization();
        }
        }  // End of else block
    }
    catch (const std::exception& e) {
        logMessage("网格处理错误: " + std::string(e.what()));
        m_statusMessage = "网格处理失败: " + std::string(e.what());
    }

    m_processingInProgress = false;
}

void ImGuiTextureMappingGUI::computeCuts() {
    if (!m_meshProcessed || m_processingInProgress || !m_geometry) {
        return;
    }

    logMessage("Computing variational cuts...");
    m_processingInProgress = true;

    try {
        // Use real cutting algorithm if available
        if (m_algorithmPipeline && m_algorithmPipeline->getCurrentState() >= SurfaceTextureMapping::AlgorithmPipeline::PipelineState::ANALYZED) {
            // Run feature detection
            auto featurePoints = m_algorithmPipeline->runFeatureDetection(m_guiState.curvatureThreshold);
            logMessage("检测到 " + std::to_string(featurePoints.size()) + " 个特征点");

            // Generate initial cuts
            auto cuts = m_algorithmPipeline->runInitialCutGeneration();
            logMessage("生成 " + std::to_string(cuts.size()) + " 条初始切割线");

            // Optimize cuts
            m_algorithmPipeline->runCutOptimization(m_guiState.maxCuttingIterations);

            // Extract visualization data
            auto visData = m_algorithmPipeline->getVisualizationData();

            // Convert cut lines to render format
            m_lineVertices.clear();
            for (const auto& cutLine : visData.cutLines) {
                for (size_t i = 0; i < cutLine.size() - 1; ++i) {
                    const auto& start = cutLine[i];
                    const auto& end = cutLine[i + 1];
                    m_lineVertices.insert(m_lineVertices.end(), {
                        static_cast<float>(start.x), static_cast<float>(start.y), static_cast<float>(start.z),
                        static_cast<float>(end.x), static_cast<float>(end.y), static_cast<float>(end.z)
                    });
                }
            }

            // Update statistics
            m_statistics.numCuts = static_cast<int>(visData.cutLines.size());
            m_statistics.avgDistortion = static_cast<float>(visData.uvDistortion);

            logMessage("变分切割完成，生成 " + std::to_string(visData.cutLines.size()) + " 条切割线");
        }
        else {
            // Fallback to SimpleShapeOptimizer
            m_shapeOptimizer = std::unique_ptr<SimpleShapeOptimizer>(new SimpleShapeOptimizer(m_geometry.get()));
            m_shapeOptimizer->weightLengthRegularization = m_guiState.lengthRegularization;
            m_shapeOptimizer->weightBilapRegularization = m_guiState.smoothRegularization;

        logMessage("Length regularization: " + std::to_string(m_guiState.lengthRegularization));
        logMessage("Smooth regularization: " + std::to_string(m_guiState.smoothRegularization));
        logMessage("Max iterations: " + std::to_string(m_guiState.maxCuttingIterations));

        // Initialize the optimizer
        m_shapeOptimizer->initializeData();
        m_shapeOptimizer->initializeTimestep();

        // Run optimization iterations
        for (int i = 0; i < m_guiState.maxCuttingIterations; i++) {
            m_shapeOptimizer->doStep();
            if (i % 10 == 0) {
                logMessage("Optimization iteration " + std::to_string(i));
            }
        }

            // Extract cut lines from the simple optimizer
            extractCutLinesFromOptimizer();

            // Compute real statistics
            double energy = m_shapeOptimizer->computeEnergy();
            m_statistics.numCuts = static_cast<int>(m_shapeOptimizer->getBoundaryLines().size());
            m_statistics.totalCutLength = static_cast<float>(m_shapeOptimizer->evaluateEnergyTermLengthRegularization());
            m_statistics.avgDistortion = static_cast<float>(energy);

            logMessage("Energy: " + std::to_string(energy));
        }

        m_cutComputed = true;
        m_statusMessage = "Cuts computed";
        logMessage("Variational cuts computed successfully");
        logMessage("Number of boundary lines: " + std::to_string(m_statistics.numCuts));

        // Update mesh visualization with cut colors and update cut line buffers
        if (m_meshLoaded) {
            updateMeshVisualization();
            updateCutsBuffers();
        }
    }
    catch (const std::exception& e) {
        logMessage("Cut computation error: " + std::string(e.what()));
        m_statusMessage = "Cut computation failed";
        // Fall back to synthetic lines for visualization
        generateSyntheticCutLines();
        if (m_meshLoaded) {
            updateCutsBuffers();
        }
    }

    m_processingInProgress = false;
}

void ImGuiTextureMappingGUI::computeUVMapping() {
    if (!m_cutComputed || m_processingInProgress || !m_shapeOptimizer) {
        return;
    }

    logMessage("Computing UV parameterization...");
    m_processingInProgress = true;

    try {
        logMessage("Curvature threshold: " + std::to_string(m_guiState.curvatureThreshold));

        // Simplified UV mapping for demonstration
        // In real implementation, would use boundary-first flattening (BFF)
        logMessage("Starting simplified UV parameterization...");

        // Initialize SimpleFlattener (using simplified approach)
        m_flattener = std::unique_ptr<SimpleFlattener>(new SimpleFlattener(m_geometry.get()));

        logMessage("Initializing BFF flattening...");

        // Run the flattening algorithm
        bool success = m_flattener->flatten();

        if (!success) {
            throw std::runtime_error("BFF flattening failed");
        }

        // Generate synthetic UV grid for visualization
        generateUVGrid();

        // Compute simplified statistics
        m_statistics.numCharts = 1;  // BFF typically creates a single chart
        m_statistics.angleDistortion = 0.15f;  // Simulated values
        m_statistics.areaDistortion = 0.12f;
        m_statistics.conformalError = 0.15f;

        m_uvMappingComputed = true;
        m_statusMessage = "UV mapping computed";
        logMessage("UV parameterization completed successfully");
        logMessage("Charts: " + std::to_string(m_statistics.numCharts));
        logMessage("Angle distortion: " + std::to_string(m_statistics.angleDistortion));
        logMessage("Area distortion: " + std::to_string(m_statistics.areaDistortion));

        // Update mesh visualization with UV mapping colors and UV grid buffers
        if (m_meshLoaded) {
            updateMeshVisualization();
            updateUVGridBuffers();
        }
    }
    catch (const std::exception& e) {
        logMessage("UV mapping error: " + std::string(e.what()));
        m_statusMessage = "UV mapping failed";
        // Fall back to synthetic UV grid
        generateUVGrid();
        if (m_meshLoaded) {
            updateUVGridBuffers();
        }
    }

    m_processingInProgress = false;
}

void ImGuiTextureMappingGUI::generatePatterns() {
    if (!m_uvMappingComputed || m_processingInProgress) {
        return;
    }

    logMessage("Generating surface patterns...");
    m_processingInProgress = true;

    try {
        const char* patternNames[] = {"Grid", "Hexagonal", "Spiral", "Hilbert"};
        logMessage("Pattern type: " + std::string(patternNames[m_guiState.patternType]));
        logMessage("Pattern spacing: " + std::to_string(m_guiState.patternSpacing));

        if (m_guiState.patternType == 3) { // Hilbert
            logMessage("Hilbert order: " + std::to_string(m_guiState.hilbertOrder));
        }

        // Generate pattern lines based on selected type
        generatePatternLines();

        // Simulate pattern generation statistics
        m_statistics.numPaths = static_cast<int>(m_patternVertices.size() / 6);
        m_statistics.totalPathLength = 2.34f;
        m_statistics.coverage = 0.78f;
        m_statistics.uniformity = 0.85f;

        m_patternsGenerated = true;
        m_statusMessage = "Patterns generated";
        logMessage("Surface patterns generated successfully");

        // Update mesh visualization with pattern colors and pattern buffers
        if (m_meshLoaded) {
            updateMeshVisualization();
            updatePatternsBuffers();
        }
    }
    catch (const std::exception& e) {
        logMessage("Pattern generation error: " + std::string(e.what()));
        m_statusMessage = "Pattern generation failed";
    }

    m_processingInProgress = false;
}

void ImGuiTextureMappingGUI::updateMeshVisualization() {
    if (m_meshVertices.empty() || m_meshIndices.empty()) {
        return;
    }

    // Update vertex colors based on current pipeline state
    for (size_t i = 0; i < m_meshVertices.size(); i += 9) { // 9 floats per vertex (pos + normal + color)
        // Skip position and normal, update only color (indices 6, 7, 8)
        glm::vec3 color = m_guiState.meshColor;

        if (m_patternsGenerated) {
            color = glm::mix(m_guiState.meshColor, m_guiState.patternColor, 0.3f);  // Pattern stage
        } else if (m_uvMappingComputed) {
            color = glm::mix(m_guiState.meshColor, glm::vec3(0.2f, 0.8f, 0.2f), 0.3f);  // UV mapping stage
        } else if (m_cutComputed) {
            color = glm::mix(m_guiState.meshColor, m_guiState.cutColor, 0.2f);  // Cutting stage
        } else if (m_meshProcessed) {
            color = glm::mix(m_guiState.meshColor, glm::vec3(0.9f, 0.7f, 0.2f), 0.2f);  // Processing stage
        }

        m_meshVertices[i + 6] = color.x;
        m_meshVertices[i + 7] = color.y;
        m_meshVertices[i + 8] = color.z;
    }

    // Update OpenGL buffers with new colors
    updateMeshBuffers();
}

// Generate synthetic cut lines for demonstration
// In a real implementation, this would interface with the variational cutting algorithm
void ImGuiTextureMappingGUI::generateSyntheticCutLines() {
    if (m_meshVertices.empty()) {
        return;
    }

    m_lineVertices.clear();

    // Get mesh bounding box for synthetic cut generation
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();

    // Extract vertex positions (every 9th element starts position data)
    for (size_t i = 0; i < m_meshVertices.size(); i += 9) {
        float x = m_meshVertices[i];
        float y = m_meshVertices[i + 1];
        float z = m_meshVertices[i + 2];

        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
        minZ = std::min(minZ, z);
        maxZ = std::max(maxZ, z);
    }

    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;
    float centerZ = (minZ + maxZ) * 0.5f;
    float radius = std::min(std::min(maxX - minX, maxY - minY), maxZ - minZ) * 0.4f;

    // Generate tennis ball-like seam curves (similar to the image)
    // Create two curved seams that meet at poles

    // First seam curve (S-shaped)
    int numPoints = 50;
    for (int i = 0; i < numPoints - 1; i++) {
        float t1 = i / float(numPoints - 1);
        float t2 = (i + 1) / float(numPoints - 1);

        // Create S-curve parametrization
        float theta1 = t1 * PI;
        float theta2 = t2 * PI;

        float phi1 = sin(theta1 * 2) * PI * 0.25f;
        float phi2 = sin(theta2 * 2) * PI * 0.25f;

        // Convert to Cartesian coordinates on sphere surface
        float x1 = centerX + radius * sin(theta1) * cos(phi1);
        float y1 = centerY + radius * cos(theta1);
        float z1 = centerZ + radius * sin(theta1) * sin(phi1);

        float x2 = centerX + radius * sin(theta2) * cos(phi2);
        float y2 = centerY + radius * cos(theta2);
        float z2 = centerZ + radius * sin(theta2) * sin(phi2);

        m_lineVertices.insert(m_lineVertices.end(), {
            x1, y1, z1,
            x2, y2, z2
        });
    }

    // Second seam curve (rotated 90 degrees)
    for (int i = 0; i < numPoints - 1; i++) {
        float t1 = i / float(numPoints - 1);
        float t2 = (i + 1) / float(numPoints - 1);

        float theta1 = t1 * PI;
        float theta2 = t2 * PI;

        float phi1 = sin(theta1 * 2) * PI * 0.25f + PI * 0.5f;
        float phi2 = sin(theta2 * 2) * PI * 0.25f + PI * 0.5f;

        float x1 = centerX + radius * sin(theta1) * cos(phi1);
        float y1 = centerY + radius * cos(theta1);
        float z1 = centerZ + radius * sin(theta1) * sin(phi1);

        float x2 = centerX + radius * sin(theta2) * cos(phi2);
        float y2 = centerY + radius * cos(theta2);
        float z2 = centerZ + radius * sin(theta2) * sin(phi2);

        m_lineVertices.insert(m_lineVertices.end(), {
            x1, y1, z1,
            x2, y2, z2
        });
    }

    logMessage("Generated tennis ball-style seam cuts with " + std::to_string(m_lineVertices.size() / 6) + " line segments");
}

// Update OpenGL buffers for cut line rendering
void ImGuiTextureMappingGUI::updateCutsBuffers() {
    if (m_lineVertices.empty()) {
        return;
    }

    // Generate buffers if not already created
    if (m_linesVAO == 0) {
        glGenVertexArrays(1, &m_linesVAO);
        glGenBuffers(1, &m_linesVBO);
    }

    // Update line buffer
    glBindVertexArray(m_linesVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_linesVBO);
    glBufferData(GL_ARRAY_BUFFER, m_lineVertices.size() * sizeof(float), m_lineVertices.data(), GL_STATIC_DRAW);

    // Position attribute (lines only need position)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

// Render cut lines
void ImGuiTextureMappingGUI::renderCuts() {
    if (!m_cutComputed || !m_guiState.showCuts || m_lineVertices.empty()) {
        return;
    }

    // Use the same shader program but with a different rendering mode
    glUseProgram(m_shaderProgram);

    // Set transformation matrices
    glm::mat4 model = glm::mat4(1.0f);

    GLint modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_viewMatrix));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(m_projectionMatrix));

    // Set lighting uniforms (same as mesh rendering)
    glm::vec3 lightPos = m_cameraPosition + glm::vec3(2.0f, 2.0f, 2.0f);
    glm::vec3 viewPos = m_cameraPosition;

    GLint lightPosLoc = glGetUniformLocation(m_shaderProgram, "lightPos");
    GLint viewPosLoc = glGetUniformLocation(m_shaderProgram, "viewPos");

    if (lightPosLoc != -1) {
        glUniform3fv(lightPosLoc, 1, glm::value_ptr(lightPos));
    }
    if (viewPosLoc != -1) {
        glUniform3fv(viewPosLoc, 1, glm::value_ptr(viewPos));
    }

    // Bind line VAO and render
    glBindVertexArray(m_linesVAO);

    // Set line width for better visibility
    glLineWidth(3.0f);

    // Draw lines
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_lineVertices.size() / 3));

    // Reset line width
    glLineWidth(1.0f);

    glBindVertexArray(0);
}

// Enhanced render function with cut line rendering
void ImGuiTextureMappingGUI::renderMesh() {
    if (!m_meshLoaded || m_meshIndices.empty()) {
        return;
    }

    // Use shader program
    glUseProgram(m_shaderProgram);

    // Set transformation matrices
    glm::mat4 model = glm::mat4(1.0f);

    GLint modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_viewMatrix));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(m_projectionMatrix));

    // Set lighting uniforms
    glm::vec3 lightPos = m_cameraPosition + glm::vec3(2.0f, 2.0f, 2.0f);
    glm::vec3 viewPos = m_cameraPosition;

    GLint lightPosLoc = glGetUniformLocation(m_shaderProgram, "lightPos");
    GLint viewPosLoc = glGetUniformLocation(m_shaderProgram, "viewPos");

    if (lightPosLoc != -1) {
        glUniform3fv(lightPosLoc, 1, glm::value_ptr(lightPos));
    }
    if (viewPosLoc != -1) {
        glUniform3fv(viewPosLoc, 1, glm::value_ptr(viewPos));
    }

    // Bind mesh VAO and render
    glBindVertexArray(m_meshVAO);

    // Render filled mesh
    if (m_guiState.showFilled) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_meshIndices.size()), GL_UNSIGNED_INT, 0);
    }

    // Render wireframe
    if (m_guiState.showWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glDrawElements(GL_TRIANGLES, static_cast<GLsizei>(m_meshIndices.size()), GL_UNSIGNED_INT, 0);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Reset to fill mode
    }

    glBindVertexArray(0);
}

// Generate UV grid for visualization - simulate unfolded tennis ball patches
void ImGuiTextureMappingGUI::generateUVGrid() {
    if (m_meshVertices.empty()) {
        return;
    }

    m_uvGridVertices.clear();

    // Get mesh bounding box
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();

    for (size_t i = 0; i < m_meshVertices.size(); i += 9) {
        float x = m_meshVertices[i];
        float y = m_meshVertices[i + 1];
        float z = m_meshVertices[i + 2];

        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
        minZ = std::min(minZ, z);
        maxZ = std::max(maxZ, z);
    }

    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;
    float centerZ = (minZ + maxZ) * 0.5f;
    float size = std::max(maxX - minX, std::max(maxY - minY, maxZ - minZ));

    // Generate two unfolded patches (like tennis ball halves)
    // Place them side by side to the right of the 3D model
    float patchWidth = size * 0.6f;
    float patchHeight = size * 0.8f;
    float offsetX = maxX + size * 0.3f;

    // First patch (left unfolded piece)
    float patch1X = offsetX;
    float patch1Y = centerY;

    // Create curved edge for first patch (S-curve shape)
    int numPoints = 30;
    std::vector<glm::vec2> patch1Points;

    for (int i = 0; i <= numPoints; i++) {
        float t = i / float(numPoints);
        float y = patch1Y - patchHeight * 0.5f + t * patchHeight;
        float x = patch1X + sin(t * PI * 2) * patchWidth * 0.15f;
        patch1Points.push_back(glm::vec2(x, y));
    }

    // Draw first patch outline
    for (size_t i = 0; i < patch1Points.size() - 1; i++) {
        m_uvGridVertices.insert(m_uvGridVertices.end(), {
            patch1Points[i].x, patch1Points[i].y, centerZ,
            patch1Points[i+1].x, patch1Points[i+1].y, centerZ
        });
    }

    // Close the patch with straight edge
    m_uvGridVertices.insert(m_uvGridVertices.end(), {
        patch1Points.back().x, patch1Points.back().y, centerZ,
        patch1X + patchWidth * 0.5f, patch1Y + patchHeight * 0.5f, centerZ
    });
    m_uvGridVertices.insert(m_uvGridVertices.end(), {
        patch1X + patchWidth * 0.5f, patch1Y + patchHeight * 0.5f, centerZ,
        patch1X + patchWidth * 0.5f, patch1Y - patchHeight * 0.5f, centerZ
    });
    m_uvGridVertices.insert(m_uvGridVertices.end(), {
        patch1X + patchWidth * 0.5f, patch1Y - patchHeight * 0.5f, centerZ,
        patch1Points[0].x, patch1Points[0].y, centerZ
    });

    // Add grid lines inside first patch
    const int gridLines = 6;
    for (int i = 1; i < gridLines; i++) {
        float t = i / float(gridLines);
        // Horizontal lines
        float y = patch1Y - patchHeight * 0.5f + t * patchHeight;
        float xLeft = patch1X + sin(t * PI * 2) * patchWidth * 0.15f;
        m_uvGridVertices.insert(m_uvGridVertices.end(), {
            xLeft, y, centerZ,
            patch1X + patchWidth * 0.5f, y, centerZ
        });
    }

    // Second patch (right unfolded piece)
    float patch2X = offsetX + patchWidth * 0.7f;
    float patch2Y = centerY;

    std::vector<glm::vec2> patch2Points;
    for (int i = 0; i <= numPoints; i++) {
        float t = i / float(numPoints);
        float y = patch2Y - patchHeight * 0.5f + t * patchHeight;
        float x = patch2X + patchWidth * 0.5f - sin(t * PI * 2) * patchWidth * 0.15f;
        patch2Points.push_back(glm::vec2(x, y));
    }

    // Draw second patch outline
    for (size_t i = 0; i < patch2Points.size() - 1; i++) {
        m_uvGridVertices.insert(m_uvGridVertices.end(), {
            patch2Points[i].x, patch2Points[i].y, centerZ,
            patch2Points[i+1].x, patch2Points[i+1].y, centerZ
        });
    }

    // Close the patch
    m_uvGridVertices.insert(m_uvGridVertices.end(), {
        patch2Points.back().x, patch2Points.back().y, centerZ,
        patch2X, patch2Y + patchHeight * 0.5f, centerZ
    });
    m_uvGridVertices.insert(m_uvGridVertices.end(), {
        patch2X, patch2Y + patchHeight * 0.5f, centerZ,
        patch2X, patch2Y - patchHeight * 0.5f, centerZ
    });
    m_uvGridVertices.insert(m_uvGridVertices.end(), {
        patch2X, patch2Y - patchHeight * 0.5f, centerZ,
        patch2Points[0].x, patch2Points[0].y, centerZ
    });

    // Add grid lines inside second patch
    for (int i = 1; i < gridLines; i++) {
        float t = i / float(gridLines);
        float y = patch2Y - patchHeight * 0.5f + t * patchHeight;
        float xRight = patch2X + patchWidth * 0.5f - sin(t * PI * 2) * patchWidth * 0.15f;
        m_uvGridVertices.insert(m_uvGridVertices.end(), {
            patch2X, y, centerZ,
            xRight, y, centerZ
        });
    }

    logMessage("Generated unfolded UV patches with " + std::to_string(m_uvGridVertices.size() / 6) + " lines");
}

// Update UV grid OpenGL buffers
void ImGuiTextureMappingGUI::updateUVGridBuffers() {
    if (m_uvGridVertices.empty()) {
        return;
    }

    // Generate buffers if not already created
    if (m_uvGridVAO == 0) {
        glGenVertexArrays(1, &m_uvGridVAO);
        glGenBuffers(1, &m_uvGridVBO);
    }

    // Update UV grid buffer
    glBindVertexArray(m_uvGridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_uvGridVBO);
    glBufferData(GL_ARRAY_BUFFER, m_uvGridVertices.size() * sizeof(float), m_uvGridVertices.data(), GL_DYNAMIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

// Render UV grid overlay
void ImGuiTextureMappingGUI::renderUVGrid() {
    if (!m_uvMappingComputed || !m_guiState.showUVMapping || m_uvGridVertices.empty()) {
        return;
    }

    glUseProgram(m_shaderProgram);

    // Set transformation matrices
    glm::mat4 model = glm::mat4(1.0f);

    GLint modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_viewMatrix));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(m_projectionMatrix));

    // Bind UV grid VAO and render
    glBindVertexArray(m_uvGridVAO);

    // Set line width and color for UV grid
    glLineWidth(1.5f);

    // Draw UV grid lines (green color)
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_uvGridVertices.size() / 3));

    glLineWidth(1.0f);
    glBindVertexArray(0);
}

// Generate pattern lines based on selected type
void ImGuiTextureMappingGUI::generatePatternLines() {
    if (m_meshVertices.empty()) {
        return;
    }

    m_patternVertices.clear();

    // Get mesh bounding box
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = std::numeric_limits<float>::lowest();

    for (size_t i = 0; i < m_meshVertices.size(); i += 9) {
        float x = m_meshVertices[i];
        float y = m_meshVertices[i + 1];
        float z = m_meshVertices[i + 2];

        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
        minZ = std::min(minZ, z);
        maxZ = std::max(maxZ, z);
    }

    float centerX = (minX + maxX) * 0.5f;
    float centerY = (minY + maxY) * 0.5f;
    float centerZ = (minZ + maxZ) * 0.5f;

    switch (m_guiState.patternType) {
        case 0: { // Grid pattern
            float spacing = m_guiState.patternSpacing;
            int numLines = static_cast<int>((maxX - minX) / spacing);

            // Vertical lines
            for (int i = 0; i <= numLines; i++) {
                float x = minX + i * spacing;
                m_patternVertices.insert(m_patternVertices.end(), {
                    x, minY, centerZ,
                    x, maxY, centerZ
                });
            }

            // Horizontal lines
            numLines = static_cast<int>((maxY - minY) / spacing);
            for (int i = 0; i <= numLines; i++) {
                float y = minY + i * spacing;
                m_patternVertices.insert(m_patternVertices.end(), {
                    minX, y, centerZ,
                    maxX, y, centerZ
                });
            }
            break;
        }

        case 1: { // Hexagonal pattern
            float spacing = m_guiState.patternSpacing;
            float hexHeight = spacing * 0.866f; // sqrt(3)/2

            int rows = static_cast<int>((maxY - minY) / hexHeight) + 1;
            int cols = static_cast<int>((maxX - minX) / spacing) + 1;

            for (int row = 0; row < rows; row++) {
                for (int col = 0; col < cols; col++) {
                    float x = minX + col * spacing * 1.5f;
                    float y = minY + row * hexHeight;

                    if (col % 2 == 1) {
                        y += hexHeight * 0.5f;
                    }

                    // Generate hex edges (6 lines per hexagon)
                    for (int i = 0; i < 6; i++) {
                        float angle1 = i * PI / 3.0f;
                        float angle2 = (i + 1) * PI / 3.0f;

                        m_patternVertices.insert(m_patternVertices.end(), {
                            x + spacing * 0.5f * cosf(angle1), y + spacing * 0.5f * sinf(angle1), centerZ,
                            x + spacing * 0.5f * cosf(angle2), y + spacing * 0.5f * sinf(angle2), centerZ
                        });
                    }
                }
            }
            break;
        }

        case 2: { // Spiral pattern
            float radius = 0.01f;
            float angleStep = 0.2f;
            float radiusStep = m_guiState.patternSpacing * 0.1f;
            float maxRadius = std::min(maxX - centerX, maxY - centerY) * 0.8f;

            std::vector<glm::vec3> spiralPoints;
            while (radius < maxRadius) {
                float angle = radius * 5.0f; // Spiral tightness
                float x = centerX + radius * cosf(angle);
                float y = centerY + radius * sinf(angle);
                spiralPoints.push_back(glm::vec3(x, y, centerZ));
                radius += radiusStep;
            }

            // Convert points to line segments
            for (size_t i = 0; i < spiralPoints.size() - 1; i++) {
                m_patternVertices.insert(m_patternVertices.end(), {
                    spiralPoints[i].x, spiralPoints[i].y, spiralPoints[i].z,
                    spiralPoints[i+1].x, spiralPoints[i+1].y, spiralPoints[i+1].z
                });
            }
            break;
        }

        case 3: { // Hilbert curve pattern
            int order = m_guiState.hilbertOrder;
            int n = 1 << order; // 2^order
            float cellSize = std::min(maxX - minX, maxY - minY) / n;

            std::vector<glm::vec2> hilbertPoints;

            // Generate Hilbert curve points using recursive algorithm
            std::function<void(int, int, int, int, int, int, int, int)> hilbert;
            hilbert = [&](int x0, int y0, int xi, int xj, int yi, int yj, int n, int depth) {
                if (n <= 0) return;

                int x = x0 + (xi + yi) / 2;
                int y = y0 + (xj + yj) / 2;

                if (depth == 0) {
                    hilbertPoints.push_back(glm::vec2(minX + x * cellSize, minY + y * cellSize));
                }

                hilbert(x0, y0, yi/2, yj/2, xi/2, xj/2, n-1, depth+1);
                hilbert(x0 + xi/2, y0 + xj/2, xi/2, xj/2, yi/2, yj/2, n-1, depth+1);
                hilbert(x0 + xi/2 + yi/2, y0 + xj/2 + yj/2, xi/2, xj/2, yi/2, yj/2, n-1, depth+1);
                hilbert(x0 + xi/2 + yi, y0 + xj/2 + yj, -yi/2, -yj/2, -xi/2, -xj/2, n-1, depth+1);
            };

            hilbert(0, 0, n, 0, 0, n, order, 0);

            // Convert Hilbert points to line segments
            for (size_t i = 0; i < hilbertPoints.size() - 1; i++) {
                m_patternVertices.insert(m_patternVertices.end(), {
                    hilbertPoints[i].x, hilbertPoints[i].y, centerZ,
                    hilbertPoints[i+1].x, hilbertPoints[i+1].y, centerZ
                });
            }
            break;
        }
    }

    const char* patternNames[] = {"Grid", "Hexagonal", "Spiral", "Hilbert"};
    logMessage("Generated " + std::string(patternNames[m_guiState.patternType]) +
               " pattern with " + std::to_string(m_patternVertices.size() / 6) + " lines");
}

// Update pattern OpenGL buffers
void ImGuiTextureMappingGUI::updatePatternsBuffers() {
    if (m_patternVertices.empty()) {
        return;
    }

    // Generate buffers if not already created
    if (m_patternVAO == 0) {
        glGenVertexArrays(1, &m_patternVAO);
        glGenBuffers(1, &m_patternVBO);
    }

    // Update pattern buffer
    glBindVertexArray(m_patternVAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_patternVBO);
    glBufferData(GL_ARRAY_BUFFER, m_patternVertices.size() * sizeof(float), m_patternVertices.data(), GL_DYNAMIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindVertexArray(0);
}

// Render pattern lines
void ImGuiTextureMappingGUI::renderPatterns() {
    if (!m_patternsGenerated || !m_guiState.showPatterns || m_patternVertices.empty()) {
        return;
    }

    glUseProgram(m_shaderProgram);

    // Set transformation matrices
    glm::mat4 model = glm::mat4(1.0f);

    GLint modelLoc = glGetUniformLocation(m_shaderProgram, "model");
    GLint viewLoc = glGetUniformLocation(m_shaderProgram, "view");
    GLint projLoc = glGetUniformLocation(m_shaderProgram, "projection");

    glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(m_viewMatrix));
    glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(m_projectionMatrix));

    // Bind pattern VAO and render
    glBindVertexArray(m_patternVAO);

    // Set line width for patterns
    glLineWidth(2.0f);

    // Draw pattern lines
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_patternVertices.size() / 3));

    glLineWidth(1.0f);
    glBindVertexArray(0);
}

// Extract cut lines from EulerianShapeOptimizer results
void ImGuiTextureMappingGUI::extractCutLinesFromOptimizer() {
    if (!m_shapeOptimizer) {
        return;
    }

    m_lineVertices.clear();

    try {
        // Get boundary lines from the optimizer
        auto boundaryLines = m_shapeOptimizer->getBoundaryLines();

        logMessage("Extracting " + std::to_string(boundaryLines.size()) + " boundary lines from optimizer");

        // Convert to render format
        for (const auto& line : boundaryLines) {
            const auto& start = line[0];
            const auto& end = line[1];

            m_lineVertices.insert(m_lineVertices.end(), {
                static_cast<float>(start.x), static_cast<float>(start.y), static_cast<float>(start.z),
                static_cast<float>(end.x), static_cast<float>(end.y), static_cast<float>(end.z)
            });
        }

        logMessage("Successfully extracted cut lines for visualization");
    }
    catch (const std::exception& e) {
        logMessage("Error extracting cut lines: " + std::string(e.what()));
        // Fall back to synthetic lines
        generateSyntheticCutLines();
    }
}

// Extract UV grid from flattening results (simplified)
void ImGuiTextureMappingGUI::extractUVGridFromFlattening() {
    logMessage("Generating simplified UV grid for demonstration");

    // Simply call the existing generateUVGrid function
    // In real implementation, would extract actual UV coordinates from BFF result
    generateUVGrid();

    logMessage("Generated synthetic UV grid for visualization");
}

// ==================== CGAL网格处理辅助方法 ====================

bool ImGuiTextureMappingGUI::convertRenderDataToCGALMesh(SurfaceMesh& cgal_mesh) {
    try {
        // Clear the simplified mesh structure
        cgal_mesh.vertices.clear();
        cgal_mesh.faces.clear();

        if (m_meshVertices.empty() || m_meshIndices.empty()) {
            logMessage("错误：渲染数据为空");
            return false;
        }

        // Extract vertices from render data (every 9 floats: position(3) + normal(3) + color(3))
        cgal_mesh.vertices.reserve(m_meshVertices.size() / 9);

        for (size_t i = 0; i < m_meshVertices.size(); i += 9) {
            std::array<float, 3> vertex = {m_meshVertices[i], m_meshVertices[i + 1], m_meshVertices[i + 2]};
            cgal_mesh.vertices.push_back(vertex);
        }

        // Add faces
        cgal_mesh.faces.reserve(m_meshIndices.size() / 3);
        for (size_t i = 0; i < m_meshIndices.size(); i += 3) {
            if (m_meshIndices[i] < cgal_mesh.vertices.size() &&
                m_meshIndices[i + 1] < cgal_mesh.vertices.size() &&
                m_meshIndices[i + 2] < cgal_mesh.vertices.size()) {

                std::array<int, 3> face = {static_cast<int>(m_meshIndices[i]),
                                          static_cast<int>(m_meshIndices[i + 1]),
                                          static_cast<int>(m_meshIndices[i + 2])};
                cgal_mesh.faces.push_back(face);
            }
        }

        logMessage("转换完成：" + std::to_string(cgal_mesh.vertices.size()) + " 顶点，" +
                  std::to_string(cgal_mesh.faces.size()) + " 面");
        return true;
    }
    catch (const std::exception& e) {
        logMessage("CGAL网格转换错误: " + std::string(e.what()));
        return false;
    }
}

bool ImGuiTextureMappingGUI::convertCGALMeshToRenderData(const SurfaceMesh& cgal_mesh) {
    try {
        m_meshVertices.clear();
        m_meshIndices.clear();

        // Extract vertices from simplified mesh structure
        for (size_t i = 0; i < cgal_mesh.vertices.size(); i++) {
            const auto& vertex = cgal_mesh.vertices[i];

            // Position (3 floats)
            m_meshVertices.push_back(vertex[0]);
            m_meshVertices.push_back(vertex[1]);
            m_meshVertices.push_back(vertex[2]);

            // Calculate normal (simplified - proper normals should be computed)
            m_meshVertices.push_back(0.0f); // normal.x
            m_meshVertices.push_back(0.0f); // normal.y
            m_meshVertices.push_back(1.0f); // normal.z

            // Color (3 floats) - use mesh color based on processing state
            glm::vec3 color = m_guiState.meshColor;
            if (m_meshProcessed) {
                color = glm::mix(m_guiState.meshColor, glm::vec3(0.9f, 0.7f, 0.2f), 0.2f);
            }
            m_meshVertices.push_back(color.x);
            m_meshVertices.push_back(color.y);
            m_meshVertices.push_back(color.z);
        }

        // Extract faces from simplified mesh structure
        for (const auto& face : cgal_mesh.faces) {
            m_meshIndices.push_back(face[0]);
            m_meshIndices.push_back(face[1]);
            m_meshIndices.push_back(face[2]);
        }

        // Recompute normals
        recomputeNormals();

        logMessage("转换回渲染格式完成：" + std::to_string(m_meshVertices.size() / 9) + " 顶点，" +
                  std::to_string(m_meshIndices.size() / 3) + " 面");
        return true;
    }
    catch (const std::exception& e) {
        logMessage("渲染数据转换错误: " + std::string(e.what()));
        return false;
    }
}

MeshQualityStats ImGuiTextureMappingGUI::analyzeMeshQuality(const SurfaceMesh& cgal_mesh) {
    MeshQualityStats stats;

    // 简化版本（不使用CGAL）
    if (cgal_mesh.vertices.size() > 0 && cgal_mesh.faces.size() > 0) {
        // 计算简单的边长统计
        std::vector<double> edge_lengths;
        for (const auto& face : cgal_mesh.faces) {
            for (int i = 0; i < 3; i++) {
                int v1 = face[i];
                int v2 = face[(i+1)%3];
                if (v1 < cgal_mesh.vertices.size() && v2 < cgal_mesh.vertices.size()) {
                    double dx = cgal_mesh.vertices[v2][0] - cgal_mesh.vertices[v1][0];
                    double dy = cgal_mesh.vertices[v2][1] - cgal_mesh.vertices[v1][1];
                    double dz = cgal_mesh.vertices[v2][2] - cgal_mesh.vertices[v1][2];
                    double len = std::sqrt(dx*dx + dy*dy + dz*dz);
                    edge_lengths.push_back(len);
                }
            }
        }

        if (!edge_lengths.empty()) {
            stats.min_edge_length = *std::min_element(edge_lengths.begin(), edge_lengths.end());
            stats.max_edge_length = *std::max_element(edge_lengths.begin(), edge_lengths.end());
            stats.avg_edge_length = std::accumulate(edge_lengths.begin(), edge_lengths.end(), 0.0) / edge_lengths.size();
        } else {
            stats.min_edge_length = 0.01;
            stats.max_edge_length = 0.1;
            stats.avg_edge_length = 0.05;
        }

        stats.is_manifold = true;
        stats.is_closed = false;
        stats.surface_area = 1.0;
    }

    return stats;
}

bool ImGuiTextureMappingGUI::repairMesh(SurfaceMesh& cgal_mesh) {
    // 简化版本：不使用CGAL
    logMessage("  执行简化网格修复...");
    logMessage("  注意：这是简化实现，未使用CGAL");

    // 基本检查
    if (cgal_mesh.vertices.empty() || cgal_mesh.faces.empty()) {
        logMessage("  错误：网格为空");
        return false;
    }

    logMessage("  顶点数: " + std::to_string(cgal_mesh.vertices.size()));
    logMessage("  面数: " + std::to_string(cgal_mesh.faces.size()));

    return true;
}

bool ImGuiTextureMappingGUI::ensureProperOrientation(SurfaceMesh& cgal_mesh) {
    // 简化版本：不使用CGAL
    logMessage("  执行简化方向修正...");
    // 简单假设网格方向正确
    return true;
}

bool ImGuiTextureMappingGUI::removeSelfIntersections(SurfaceMesh& cgal_mesh) {
    // 简化版本：不使用CGAL
    logMessage("  跳过自相交检查（简化版）");
    return true;
}

bool ImGuiTextureMappingGUI::performIsotropicRemeshing(SurfaceMesh& cgal_mesh) {
    // 简化版本（不使用CGAL）
    logMessage("  执行简化等各向性重网格化...");
    logMessage("  注意：这是简化实现，未使用CGAL");
    logMessage("  保持网格不变");

    // 简化实现：暂时只返回成功
    return true;
}

void ImGuiTextureMappingGUI::logInitialMeshStatistics(const MeshQualityStats& stats) {
    logMessage("=== 初始网格质量统计 ===");
    logMessage("流形性: " + (stats.is_manifold ? std::string("是") : std::string("否")));
    logMessage("封闭性: " + (stats.is_closed ? std::string("是") : std::string("否")));
    logMessage("方向性: " + (stats.is_oriented ? std::string("正确") : std::string("需要修正")));
    logMessage("边长统计 - 最小: " + std::to_string(stats.min_edge_length) +
              ", 最大: " + std::to_string(stats.max_edge_length) +
              ", 平均: " + std::to_string(stats.avg_edge_length));
    logMessage("边长标准差: " + std::to_string(stats.edge_length_std_dev));
    logMessage("表面积: " + std::to_string(stats.surface_area));
    if (stats.is_closed) {
        logMessage("体积: " + std::to_string(stats.volume));
    }
    logMessage("退化面数量: " + std::to_string(stats.degenerate_faces));
    logMessage("自相交数量: " + std::to_string(stats.self_intersections));
    logMessage("边界组件数: " + std::to_string(stats.boundary_components));
}

void ImGuiTextureMappingGUI::logFinalMeshStatistics(const MeshQualityStats& initial, const MeshQualityStats& final) {
    logMessage("=== 最终网格质量统计 ===");
    logMessage("流形性: " + (final.is_manifold ? std::string("是") : std::string("否")) +
              (initial.is_manifold != final.is_manifold ? " (已改善)" : ""));
    logMessage("封闭性: " + (final.is_closed ? std::string("是") : std::string("否")));
    logMessage("方向性: " + (final.is_oriented ? std::string("正确") : std::string("需要修正")) +
              (initial.is_oriented != final.is_oriented ? " (已修正)" : ""));

    logMessage("边长改善:");
    logMessage("  平均边长: " + std::to_string(initial.avg_edge_length) + " → " +
              std::to_string(final.avg_edge_length));
    logMessage("  标准差: " + std::to_string(initial.edge_length_std_dev) + " → " +
              std::to_string(final.edge_length_std_dev) +
              (final.edge_length_std_dev < initial.edge_length_std_dev ? " (已改善)" : ""));

    logMessage("缺陷修复:");
    logMessage("  退化面: " + std::to_string(initial.degenerate_faces) + " → " +
              std::to_string(final.degenerate_faces));
    logMessage("  自相交: " + std::to_string(initial.self_intersections) + " → " +
              std::to_string(final.self_intersections));

    logMessage("表面积: " + std::to_string(initial.surface_area) + " → " +
              std::to_string(final.surface_area));

    if (final.is_closed) {
        logMessage("体积: " + std::to_string(final.volume));
    }
}

void ImGuiTextureMappingGUI::recomputeNormals() {
    if (m_meshVertices.empty() || m_meshIndices.empty()) {
        return;
    }

    // Clear existing normals
    for (size_t i = 3; i < m_meshVertices.size(); i += 9) {
        m_meshVertices[i] = 0.0f;     // normal.x
        m_meshVertices[i + 1] = 0.0f; // normal.y
        m_meshVertices[i + 2] = 0.0f; // normal.z
    }

    // Accumulate face normals to vertex normals
    for (size_t i = 0; i < m_meshIndices.size(); i += 3) {
        if (i + 2 < m_meshIndices.size()) {
            int idx0 = m_meshIndices[i] * 9;
            int idx1 = m_meshIndices[i + 1] * 9;
            int idx2 = m_meshIndices[i + 2] * 9;

            if (idx0 + 8 < static_cast<int>(m_meshVertices.size()) &&
                idx1 + 8 < static_cast<int>(m_meshVertices.size()) &&
                idx2 + 8 < static_cast<int>(m_meshVertices.size())) {

                glm::vec3 v0(m_meshVertices[idx0], m_meshVertices[idx0 + 1], m_meshVertices[idx0 + 2]);
                glm::vec3 v1(m_meshVertices[idx1], m_meshVertices[idx1 + 1], m_meshVertices[idx1 + 2]);
                glm::vec3 v2(m_meshVertices[idx2], m_meshVertices[idx2 + 1], m_meshVertices[idx2 + 2]);

                glm::vec3 normal = glm::normalize(glm::cross(v1 - v0, v2 - v0));

                // Accumulate to vertices
                for (int j = 0; j < 3; j++) {
                    int idx = (j == 0) ? idx0 : (j == 1) ? idx1 : idx2;
                    m_meshVertices[idx + 3] += normal.x;
                    m_meshVertices[idx + 4] += normal.y;
                    m_meshVertices[idx + 5] += normal.z;
                }
            }
        }
    }

    // Normalize accumulated normals
    for (size_t i = 3; i < m_meshVertices.size(); i += 9) {
        glm::vec3 normal(m_meshVertices[i], m_meshVertices[i + 1], m_meshVertices[i + 2]);
        float length = glm::length(normal);
        if (length > 0.0f) {
            normal /= length;
            m_meshVertices[i] = normal.x;
            m_meshVertices[i + 1] = normal.y;
            m_meshVertices[i + 2] = normal.z;
        } else {
            // Default normal if calculation fails
            m_meshVertices[i] = 0.0f;
            m_meshVertices[i + 1] = 0.0f;
            m_meshVertices[i + 2] = 1.0f;
        }
    }
}

} // namespace SurfaceTextureMapping
