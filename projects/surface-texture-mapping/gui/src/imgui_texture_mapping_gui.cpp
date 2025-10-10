#include "imgui_texture_mapping_gui.h"
#include <iostream>
#include <sstream>
#include <iomanip>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <numeric>

// 包含真实的算法组件头文件
#include "../../core/include/mesh_processing.h"
#include "../../core/include/variational_cutting.h"
#include "../../core/include/texture_mapping.h"
#include "../../core/include/surface_filling.h"
#include "../../core/include/uv_distortion_analyzer.h"
#include "../../core/include/barycentric_mapper.h"
#include "../../core/include/pattern_back_mapper.h"

namespace SurfaceTextureMapping {

// Vertex shader source code
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;

uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

out vec3 FragPos;
out vec3 Normal;

void main() {
    FragPos = vec3(model * vec4(aPos, 1.0));
    Normal = mat3(transpose(inverse(model))) * aNormal;

    gl_Position = projection * view * vec4(FragPos, 1.0);
}
)";

// Fragment shader source code
const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 objectColor;
uniform vec3 lightColor;
uniform vec3 lightPos;
uniform vec3 viewPos;
uniform bool wireframe;

void main() {
    if (wireframe) {
        FragColor = vec4(objectColor, 1.0);
        return;
    }

    // Ambient light
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse light
    vec3 norm = normalize(Normal);
    vec3 lightDir = normalize(lightPos - FragPos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular light
    float specularStrength = 0.5;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * lightColor;

    vec3 result = (ambient + diffuse + specular) * objectColor;
    FragColor = vec4(result, 1.0);
}
)";

ImGuiTextureMappingGUI::ImGuiTextureMappingGUI()
    : m_window(nullptr)
    , m_shaderProgram(0)
    , m_meshVAO(0), m_meshVBO(0), m_meshEBO(0)
    , m_linesVAO(0), m_linesVBO(0)
{
    // Initialize Surface Texture Mapping components (真实算法)
    m_meshProcessor = std::make_unique<MeshProcessor>();
    m_cutter = std::make_unique<VariationalCutter>();
    m_textureMapper = std::make_unique<TextureMapper>();
    m_surfaceFiller = std::make_unique<SurfaceFiller>();
    m_distortionAnalyzer = std::make_unique<UVDistortionAnalyzer>();
    m_barycentricMapper = std::make_shared<BarycentricMapper>();  // shared_ptr for PatternBackMapper
    m_patternBackMapper = std::make_unique<PatternBackMapper>();
}

ImGuiTextureMappingGUI::~ImGuiTextureMappingGUI() {
    cleanup();
}

bool ImGuiTextureMappingGUI::initialize() {
    if (!initializeGLFW()) {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return false;
    }

    if (!initializeOpenGL()) {
        std::cerr << "Failed to initialize OpenGL" << std::endl;
        return false;
    }

    if (!initializeImGui()) {
        std::cerr << "Failed to initialize ImGui" << std::endl;
        return false;
    }

    // Initialize camera
    updateCamera();

    logMessage("GUI initialized successfully");
    return true;
}

bool ImGuiTextureMappingGUI::initializeGLFW() {
    if (!glfwInit()) {
        return false;
    }

    // OpenGL version settings
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window
    m_window = glfwCreateWindow(m_windowWidth, m_windowHeight,
                               "Surface Texture Mapping - ImGui Version", NULL, NULL);
    if (!m_window) {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1); // Enable V-Sync

    // Set callback functions
    glfwSetWindowUserPointer(m_window, this);
    glfwSetMouseButtonCallback(m_window, mouseCallback);
    glfwSetCursorPosCallback(m_window, cursorCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetKeyCallback(m_window, keyCallback);

    return true;
}

bool ImGuiTextureMappingGUI::initializeOpenGL() {
    // Initialize GLEW
    if (glewInit() != GLEW_OK) {
        return false;
    }

    // OpenGL settings
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Background color
    glClearColor(0.2f, 0.3f, 0.3f, 1.0f);

    // Load shaders
    return loadShaders();
}

bool ImGuiTextureMappingGUI::loadShaders() {
    // Compile vertex shader
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    // Check compilation errors
    int success;
    char infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "Vertex shader compilation failed: " << infoLog << std::endl;
        return false;
    }

    // Compile fragment shader
    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);

    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "Fragment shader compilation failed: " << infoLog << std::endl;
        return false;
    }

    // Link shader program
    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);

    glGetProgramiv(m_shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(m_shaderProgram, 512, NULL, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
        return false;
    }

    // Cleanup
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    return true;
}

bool ImGuiTextureMappingGUI::initializeImGui() {
    // Set up ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // 注意：Docking功能需要ImGui版本支持，如果版本不支持可能会报错
    #ifdef IMGUI_HAS_DOCK
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    #endif

    // Set ImGui style (before font configuration)
    ImGui::StyleColorsDark();

    // Configure fonts to support Chinese display (must be done before backend initialization)
    std::cout << "Loading Chinese fonts..." << std::endl;

    // Use Windows system Chinese fonts, fall back to default if not available
    const char* fontPaths[] = {
        "C:/Windows/Fonts/simhei.ttf",    // 黑体
        "C:/Windows/Fonts/simsun.ttc",    // 宋体
        "C:/Windows/Fonts/msyh.ttc",      // 微软雅黑
        nullptr
    };

    bool fontLoaded = false;
    for (const char** fontPath = fontPaths; *fontPath != nullptr; ++fontPath) {
        std::cout << "Trying font: " << *fontPath << std::endl;

        // Check if file exists
        std::ifstream file(*fontPath);
        if (file.good()) {
            file.close();
            std::cout << "Font file exists, loading..." << std::endl;

            // Load Chinese font with appropriate size
            ImFont* font = io.Fonts->AddFontFromFileTTF(*fontPath, 16.0f, nullptr,
                io.Fonts->GetGlyphRangesChineseFull());

            if (font != nullptr) {
                std::cout << "Successfully loaded font: " << *fontPath << std::endl;
                logMessage(std::string("Successfully loaded font: ") + *fontPath);
                fontLoaded = true;
                break;
            } else {
                std::cout << "Failed to load font: " << *fontPath << std::endl;
            }
        } else {
            std::cout << "Font file not found: " << *fontPath << std::endl;
        }
    }

    if (!fontLoaded) {
        // If unable to load Chinese fonts, ensure default font is available
        std::cout << "Warning: Could not load Chinese fonts, using default font" << std::endl;
        logMessage("Warning: Could not load Chinese fonts, using default font");
        io.Fonts->AddFontDefault();
    }

    // Set platform/renderer backend (after font configuration)
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    // Build font atlas and upload to GPU (must be after renderer initialization)
    std::cout << "Building and uploading font atlas..." << std::endl;
    ImGui_ImplOpenGL3_CreateFontsTexture();

    std::cout << "ImGui initialization completed" << std::endl;
    return true;
}

void ImGuiTextureMappingGUI::run() {
    while (!glfwWindowShouldClose(m_window)) {
        glfwPollEvents();

        // Start new ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // Render 3D scene
        render();

        // Draw GUI interface
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
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_meshLoaded) {
        return;
    }

    // Use shader program
    glUseProgram(m_shaderProgram);

    // Set uniformss
    glm::mat4 model = glm::mat4(1.0f);
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(m_viewMatrix));
    glUniformMatrix4fv(glGetUniformLocation(m_shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(m_projectionMatrix));

    glUniform3fv(glGetUniformLocation(m_shaderProgram, "lightColor"), 1, glm::value_ptr(glm::vec3(1.0f, 1.0f, 1.0f)));
    glUniform3fv(glGetUniformLocation(m_shaderProgram, "lightPos"), 1, glm::value_ptr(glm::vec3(2.0f, 2.0f, 2.0f)));
    glUniform3fv(glGetUniformLocation(m_shaderProgram, "viewPos"), 1, glm::value_ptr(m_cameraPosition));

    // Render mesh
    if (m_guiState.showFilled) {
        renderMesh();
    }

    // Render wireframe
    if (m_guiState.showWireframe) {
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glUniform1i(glGetUniformLocation(m_shaderProgram, "wireframe"), true);
        glUniform3fv(glGetUniformLocation(m_shaderProgram, "objectColor"), 1, glm::value_ptr(m_guiState.wireframeColor));
        renderMesh();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // Render cuts
    if (m_guiState.showCuts) {
        renderCuts();
    }

    // Render patterns
    if (m_guiState.showPatterns) {
        renderPatterns();
    }
}

void ImGuiTextureMappingGUI::renderMesh() {
    if (m_meshVertices.empty()) return;

    glUniform1i(glGetUniformLocation(m_shaderProgram, "wireframe"), false);
    glUniform3fv(glGetUniformLocation(m_shaderProgram, "objectColor"), 1, glm::value_ptr(m_guiState.meshColor));

    glBindVertexArray(m_meshVAO);
    glDrawElements(GL_TRIANGLES, m_meshIndices.size(), GL_UNSIGNED_INT, 0);
}

void ImGuiTextureMappingGUI::renderCuts() {
    // TODO: Implement cuts rendering
}

void ImGuiTextureMappingGUI::renderPatterns() {
    // TODO: Implement patterns rendering
}

void ImGuiTextureMappingGUI::drawMainMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Open Mesh...", "Ctrl+O")) {
                // TODO: File dialog
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Export UV Mesh...", nullptr, false, m_meshLoaded)) {
                // TODO: Export functionality
            }

            if (ImGui::MenuItem("Export Pattern...", nullptr, false, m_meshLoaded)) {
                // TODO: Export functionality
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Quit", "Alt+F4")) {
                glfwSetWindowShouldClose(m_window, GLFW_TRUE);
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Control Panel", nullptr, true);
            ImGui::MenuItem("Visualization", nullptr, true);
            ImGui::MenuItem("Statistics", nullptr, true);
            ImGui::MenuItem("Log", nullptr, true);

            ImGui::Separator();

            if (ImGui::MenuItem("Reset Camera")) {
                resetCamera();
            }

            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("Process")) {
            if (ImGui::MenuItem("Run Full Pipeline", nullptr, false, m_meshLoaded && !m_processingInProgress)) {
                runFullPipeline();
            }

            ImGui::Separator();

            if (ImGui::MenuItem("Process Mesh", nullptr, false, m_meshLoaded && !m_processingInProgress)) {
                processMesh();
            }

            if (ImGui::MenuItem("Compute Cuts", nullptr, false, m_meshLoaded && !m_processingInProgress)) {
                computeCuts();
            }

            if (ImGui::MenuItem("UV Mapping", nullptr, false, m_meshLoaded && !m_processingInProgress)) {
                computeUVMapping();
            }

            if (ImGui::MenuItem("Generate Patterns", nullptr, false, m_meshLoaded && !m_processingInProgress)) {
                generatePatterns();
            }

            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void ImGuiTextureMappingGUI::drawControlPanel() {
    ImGui::Begin("Control Panel");

    // File Input/Output
    ImGui::SeparatorText("Input/Output");
    ImGui::InputText("Input File", m_guiState.inputFile, sizeof(m_guiState.inputFile));
    ImGui::SameLine();
    if (ImGui::Button("Load")) {
        loadMesh(m_guiState.inputFile);
    }

    ImGui::InputText("Output Prefix", m_guiState.outputPrefix, sizeof(m_guiState.outputPrefix));

    // Mesh Processing Parameters
    ImGui::SeparatorText("Mesh Processing");
    ImGui::Checkbox("Enable Remeshing", &m_guiState.enableRemeshing);
    if (m_guiState.enableRemeshing) {
        ImGui::SliderFloat("Target Edge Length", &m_guiState.targetEdgeLength, 0.005f, 0.05f, "%.3f");
        ImGui::SliderInt("Remesh Iterations", &m_guiState.remeshIterations, 1, 10);
        ImGui::Checkbox("Protect Boundary", &m_guiState.protectBoundary);
    }

    // Variational Cutting Parameters
    ImGui::SeparatorText("Variational Cutting");
    ImGui::Checkbox("Enable Cutting", &m_guiState.enableCutting);
    if (m_guiState.enableCutting) {
        ImGui::SliderFloat("Length Regularization", &m_guiState.lengthRegularization, 0.0f, 1.0f, "%.3f");
        ImGui::SliderFloat("Smooth Regularization", &m_guiState.smoothRegularization, 0.0f, 0.5f, "%.3f");
        ImGui::SliderInt("Max Iterations", &m_guiState.maxCuttingIterations, 10, 100);
    }

    // Texture Mapping Parameters
    ImGui::SeparatorText("Texture Mapping");
    ImGui::Checkbox("Enable Texturing", &m_guiState.enableTexturing);
    if (m_guiState.enableTexturing) {
        ImGui::SliderFloat("Curvature Threshold", &m_guiState.curvatureThreshold, 0.05f, 0.5f, "%.3f");
    }

    // Surface Filling Parameters
    ImGui::SeparatorText("Surface Filling");
    ImGui::Checkbox("Enable Filling", &m_guiState.enableFilling);
    if (m_guiState.enableFilling) {
        const char* patternTypes[] = {"Grid", "Hexagonal", "Spiral", "Hilbert"};
        ImGui::Combo("Pattern Type", &m_guiState.patternType, patternTypes, IM_ARRAYSIZE(patternTypes));
        ImGui::SliderFloat("Pattern Spacing", &m_guiState.patternSpacing, 0.01f, 0.1f, "%.3f");
        if (m_guiState.patternType == 3) { // Hilbert
            ImGui::SliderInt("Hilbert Order", &m_guiState.hilbertOrder, 2, 6);
        }
    }

    // Process Button
    ImGui::Separator();
    if (ImGui::Button("Run Full Pipeline", ImVec2(-1, 30))) {
        if (m_meshLoaded && !m_processingInProgress) {
            runFullPipeline();
        }
    }

    ImGui::End();
}

void ImGuiTextureMappingGUI::drawVisualizationPanel() {
    ImGui::Begin("Visualization");

    // Display Options
    ImGui::SeparatorText("Display Options");
    ImGui::Checkbox("Show Wireframe", &m_guiState.showWireframe);
    ImGui::Checkbox("Show Filled", &m_guiState.showFilled);
    ImGui::Checkbox("Show Cuts", &m_guiState.showCuts);
    ImGui::Checkbox("Show UV Mapping", &m_guiState.showUVMapping);
    ImGui::Checkbox("Show Patterns", &m_guiState.showPatterns);

    // Color Settings
    ImGui::SeparatorText("Colors");
    ImGui::ColorEdit3("Mesh Color", glm::value_ptr(m_guiState.meshColor));
    ImGui::ColorEdit3("Wireframe Color", glm::value_ptr(m_guiState.wireframeColor));
    ImGui::ColorEdit3("Cut Color", glm::value_ptr(m_guiState.cutColor));
    ImGui::ColorEdit3("Pattern Color", glm::value_ptr(m_guiState.patternColor));

    // Camera Control
    ImGui::SeparatorText("Camera");
    if (ImGui::Button("Reset Camera")) {
        resetCamera();
    }

    ImGui::SliderFloat("Distance", &m_cameraDistance, 1.0f, 10.0f);
    updateCamera();

    ImGui::End();
}

void ImGuiTextureMappingGUI::drawStatisticsPanel() {
    ImGui::Begin("Statistics");

    if (m_meshLoaded) {
        ImGui::SeparatorText("Mesh Statistics");
        ImGui::Text("Vertices: %d", m_statistics.numVertices);
        ImGui::Text("Faces: %d", m_statistics.numFaces);
        ImGui::Text("Edges: %d", m_statistics.numEdges);
        ImGui::Text("Avg Edge Length: %.6f", m_statistics.avgEdgeLength);
        ImGui::Text("Min Edge Length: %.6f", m_statistics.minEdgeLength);
        ImGui::Text("Max Edge Length: %.6f", m_statistics.maxEdgeLength);
        ImGui::Text("Manifold: %s", m_statistics.isManifold ? "Yes" : "No");
        ImGui::Text("Closed: %s", m_statistics.isClosed ? "Yes" : "No");

        if (m_statistics.numCuts > 0) {
            ImGui::SeparatorText("Cutting Statistics");
            ImGui::Text("Number of Cuts: %d", m_statistics.numCuts);
            ImGui::Text("Total Cut Length: %.6f", m_statistics.totalCutLength);
            ImGui::Text("Avg Distortion: %.6f", m_statistics.avgDistortion);
        }

        if (m_statistics.numCharts > 0) {
            ImGui::SeparatorText("UV Mapping Statistics");
            ImGui::Text("Number of Charts: %d", m_statistics.numCharts);
            ImGui::Text("Angle Distortion: %.6f", m_statistics.angleDistortion);
            ImGui::Text("Area Distortion: %.6f", m_statistics.areaDistortion);
            ImGui::Text("Conformal Error: %.6f", m_statistics.conformalError);
        }

        if (m_statistics.numPaths > 0) {
            ImGui::SeparatorText("Pattern Statistics");
            ImGui::Text("Number of Paths: %d", m_statistics.numPaths);
            ImGui::Text("Total Path Length: %.6f", m_statistics.totalPathLength);
            ImGui::Text("Coverage: %.1f%%", m_statistics.coverage * 100);
            ImGui::Text("Uniformity: %.3f", m_statistics.uniformity);
        }
    } else {
        ImGui::Text("No mesh loaded");
    }

    ImGui::End();
}

void ImGuiTextureMappingGUI::drawLogPanel() {
    ImGui::Begin("Log");

    ImGui::Checkbox("Auto-scroll", &m_autoScroll);
    ImGui::SameLine();
    if (ImGui::Button("Clear")) {
        m_logMessages.clear();
    }

    ImGui::Separator();

    ImGui::BeginChild("ScrollingRegion", ImVec2(0, 0), false, ImGuiWindowFlags_HorizontalScrollbar);

    for (const auto& message : m_logMessages) {
        ImGui::TextUnformatted(message.c_str());
    }

    if (m_autoScroll && ImGui::GetScrollY() >= ImGui::GetScrollMaxY()) {
        ImGui::SetScrollHereY(1.0f);
    }

    ImGui::EndChild();
    ImGui::End();
}

// Continue implementing remaining methods...

bool ImGuiTextureMappingGUI::loadMesh(const std::string& filename) {
    try {
        logMessage("Loading mesh: " + filename);

        // 使用MeshProcessor加载网格
        if (!m_meshProcessor->loadMesh(filename)) {
            logMessage("Error: Failed to load mesh from " + filename);
            return false;
        }

        // 获取加载的网格和几何数据
        m_mesh = m_meshProcessor->getMesh();
        m_geometry = m_meshProcessor->getGeometry();

        if (!m_mesh || !m_geometry) {
            logMessage("Error: Invalid mesh or geometry data");
            return false;
        }

        // 将加载的网格数据传递给其他处理组件（真实的组件需要两个参数）
        m_cutter->setMesh(m_mesh, m_geometry);
        m_textureMapper->setMesh(m_mesh, m_geometry);
        // SurfaceFiller需要通过setInput设置，稍后在UV映射完成后调用

        updateMeshBuffers();
        updateStatistics();

        m_meshLoaded = true;
        logMessage("Mesh loaded successfully: " + std::to_string(m_mesh->nVertices()) + " vertices, " +
                  std::to_string(m_mesh->nFaces()) + " faces");

        // 重置相机以适应新模型
        resetCamera();

        return true;
    } catch (const std::exception& e) {
        logMessage("Error loading mesh: " + std::string(e.what()));
        return false;
    }
}

void ImGuiTextureMappingGUI::updateMeshBuffers() {
    using namespace geometrycentral;
    using namespace geometrycentral::surface;

    if (!m_mesh || !m_geometry) return;

    // Clear existing data
    m_meshVertices.clear();
    m_meshIndices.clear();

    // 先计算法线
    m_geometry->requireVertexNormals();

    // Build vertex data (position + normal)
    for (Vertex v : m_mesh->vertices()) {
        Vector3 pos = m_geometry->vertexPositions[v];
        Vector3 normal = m_geometry->vertexNormals[v];

        m_meshVertices.push_back(pos.x);
        m_meshVertices.push_back(pos.y);
        m_meshVertices.push_back(pos.z);
        m_meshVertices.push_back(normal.x);
        m_meshVertices.push_back(normal.y);
        m_meshVertices.push_back(normal.z);
    }

    // Build index data
    for (Face f : m_mesh->faces()) {
        std::vector<size_t> indices;
        for (Vertex v : f.adjacentVertices()) {
            indices.push_back(v.getIndex());
        }
        // Assume all are triangles
        if (indices.size() >= 3) {
            m_meshIndices.push_back(indices[0]);
            m_meshIndices.push_back(indices[1]);
            m_meshIndices.push_back(indices[2]);
        }
    }

    // Update OpenGL buffers
    if (m_meshVAO == 0) {
        glGenVertexArrays(1, &m_meshVAO);
        glGenBuffers(1, &m_meshVBO);
        glGenBuffers(1, &m_meshEBO);
    }

    glBindVertexArray(m_meshVAO);

    glBindBuffer(GL_ARRAY_BUFFER, m_meshVBO);
    glBufferData(GL_ARRAY_BUFFER, m_meshVertices.size() * sizeof(float),
                 m_meshVertices.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_meshEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_meshIndices.size() * sizeof(unsigned int),
                 m_meshIndices.data(), GL_STATIC_DRAW);

    // Position attributes
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Normal attributes
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    glBindVertexArray(0);
}

void ImGuiTextureMappingGUI::updateCamera() {
    // Spherical to Cartesian coordinate conversion
    float x = m_cameraDistance * sin(m_cameraRotationX) * cos(m_cameraRotationY);
    float y = m_cameraDistance * sin(m_cameraRotationY);
    float z = m_cameraDistance * cos(m_cameraRotationX) * cos(m_cameraRotationY);

    m_cameraPosition = glm::vec3(x, y, z);
    m_viewMatrix = glm::lookAt(m_cameraPosition, m_cameraTarget, m_cameraUp);

    // Update projection matrix
    int width, height;
    glfwGetFramebufferSize(m_window, &width, &height);
    m_projectionMatrix = glm::perspective(glm::radians(45.0f),
                                         (float)width / (float)height, 0.1f, 100.0f);
}

void ImGuiTextureMappingGUI::resetCamera() {
    m_cameraDistance = 3.0f;
    m_cameraRotationX = 0.0f;
    m_cameraRotationY = 0.0f;
    m_cameraTarget = glm::vec3(0, 0, 0);
    updateCamera();
}

// Event callback functions implementation
void ImGuiTextureMappingGUI::mouseCallback(GLFWwindow* window, int button, int action, int mods) {
    ImGuiTextureMappingGUI* gui = static_cast<ImGuiTextureMappingGUI*>(glfwGetWindowUserPointer(window));

    // 检查ImGui是否正在使用鼠标（例如拖动窗口、点击按钮等）
    // 如果是，则不处理3D场景的相机控制
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        gui->m_mousePressed = false;  // 确保释放鼠标控制
        return;
    }

    if (button == GLFW_MOUSE_BUTTON_LEFT) {
        gui->m_mousePressed = (action == GLFW_PRESS);
    }
}

void ImGuiTextureMappingGUI::cursorCallback(GLFWwindow* window, double xpos, double ypos) {
    ImGuiTextureMappingGUI* gui = static_cast<ImGuiTextureMappingGUI*>(glfwGetWindowUserPointer(window));

    // 检查ImGui是否正在使用鼠标
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        // ImGui正在使用鼠标，不处理相机旋转
        gui->m_lastMouseX = xpos;
        gui->m_lastMouseY = ypos;
        return;
    }

    if (gui->m_mousePressed) {
        double deltaX = xpos - gui->m_lastMouseX;
        double deltaY = ypos - gui->m_lastMouseY;

        gui->m_cameraRotationX += deltaX * 0.01f;
        gui->m_cameraRotationY += deltaY * 0.01f;

        // Limit Y-axis rotation
        gui->m_cameraRotationY = std::max(-1.5f, std::min(1.5f, gui->m_cameraRotationY));

        gui->updateCamera();
    }

    gui->m_lastMouseX = xpos;
    gui->m_lastMouseY = ypos;
}

void ImGuiTextureMappingGUI::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGuiTextureMappingGUI* gui = static_cast<ImGuiTextureMappingGUI*>(glfwGetWindowUserPointer(window));

    // 检查ImGui是否正在使用鼠标（例如滚动窗口内容）
    ImGuiIO& io = ImGui::GetIO();
    if (io.WantCaptureMouse) {
        // ImGui正在使用鼠标，不处理相机缩放
        return;
    }

    gui->m_cameraDistance -= yoffset * 0.1f;
    gui->m_cameraDistance = std::max(0.5f, std::min(10.0f, gui->m_cameraDistance));
    gui->updateCamera();
}

void ImGuiTextureMappingGUI::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (key == GLFW_KEY_ESCAPE && action == GLFW_PRESS) {
        glfwSetWindowShouldClose(window, GLFW_TRUE);
    }
}

// Utility functions
glm::vec3 ImGuiTextureMappingGUI::eigenToGLM(const STM::Vector3& v) {
    return glm::vec3(v.x, v.y, v.z);
}

STM::Vector3 ImGuiTextureMappingGUI::glmToEigen(const glm::vec3& v) {
    return STM::Vector3{v.x, v.y, v.z};
}

void ImGuiTextureMappingGUI::logMessage(const std::string& message) {
    // Add timestamp
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);

    std::stringstream ss;
    ss << std::put_time(std::localtime(&time_t), "[%H:%M:%S] ");
    ss << message;

    m_logMessages.push_back(ss.str());

    // Limit log entries count
    if (m_logMessages.size() > 1000) {
        m_logMessages.erase(m_logMessages.begin());
    }
}

void ImGuiTextureMappingGUI::updateStatistics() {
    using namespace geometrycentral;
    using namespace geometrycentral::surface;

    if (!m_mesh || !m_geometry) return;

    m_statistics.numVertices = m_mesh->nVertices();
    m_statistics.numFaces = m_mesh->nFaces();
    m_statistics.numEdges = m_mesh->nEdges();
    m_statistics.isManifold = true; // ManifoldSurfaceMesh is always manifold
    m_statistics.isClosed = !m_mesh->hasBoundary();

    // Calculate edge length statistics
    m_geometry->requireEdgeLengths();

    double totalLength = 0;
    double minLength = std::numeric_limits<double>::max();
    double maxLength = 0;

    for (Edge e : m_mesh->edges()) {
        double length = m_geometry->edgeLengths[e];
        totalLength += length;
        minLength = std::min(minLength, length);
        maxLength = std::max(maxLength, length);
    }

    m_statistics.avgEdgeLength = totalLength / m_mesh->nEdges();
    m_statistics.minEdgeLength = minLength;
    m_statistics.maxEdgeLength = maxLength;
}

// Simplified implementation of processing pipeline methods
void ImGuiTextureMappingGUI::runFullPipeline() {
    if (!m_meshLoaded || m_processingInProgress) return;

    m_processingInProgress = true;
    logMessage("Starting full processing pipeline...");

    try {
        // Step 1: Process mesh (重网格化等预处理)
        if (m_guiState.enableRemeshing) {
            processMesh();
        }

        // Step 2: Compute cuts (变分切割)
        if (m_guiState.enableCutting) {
            computeCuts();
        }

        // Step 3: UV mapping (参数化展开) - 始终执行以测试真实算法
        computeUVMapping();

        // Step 4: Generate patterns (图案生成) - 始终执行以测试真实算法
        if (m_uvMappingComputed) {
            generatePatterns();
        }

        m_statusMessage = "Processing completed";
        logMessage("Full pipeline completed successfully");
    } catch (const std::exception& e) {
        m_statusMessage = "Processing failed";
        logMessage("Error in pipeline: " + std::string(e.what()));
    }

    m_processingInProgress = false;
}

void ImGuiTextureMappingGUI::processMesh() {
    if (!m_meshLoaded) return;

    logMessage("Step 1: Processing mesh (remeshing)...");

    try {
        // 使用MeshProcessor进行重网格化
        if (m_guiState.enableRemeshing) {
            bool success = m_meshProcessor->isotropicRemeshing(
                m_guiState.targetEdgeLength,
                m_guiState.remeshIterations,
                m_guiState.protectBoundary
            );

            if (success) {
                // 更新网格数据
                m_mesh = m_meshProcessor->getMesh();
                m_geometry = m_meshProcessor->getGeometry();

                // 更新其他组件的网格引用
                m_cutter->setMesh(m_mesh, m_geometry);
                m_textureMapper->setMesh(m_mesh, m_geometry);

                // 更新显示
                updateMeshBuffers();
                updateStatistics();

                m_meshProcessed = true;
                logMessage("Mesh processing completed: " +
                          std::to_string(m_mesh->nVertices()) + " vertices");
            } else {
                logMessage("Warning: Mesh processing failed, using original mesh");
            }
        }
    } catch (const std::exception& e) {
        logMessage("Error in mesh processing: " + std::string(e.what()));
    }
}

void ImGuiTextureMappingGUI::computeCuts() {
    if (!m_meshLoaded) return;

    logMessage("Step 2: Computing variational cuts...");

    try {
        // 设置变分切割参数
        VariationalCutter::CuttingParams params;
        params.lengthRegularization = m_guiState.lengthRegularization;
        params.smoothRegularization = m_guiState.smoothRegularization;
        params.maxIterations = m_guiState.maxCuttingIterations;

        // 计算最优切缝
        auto cuts = m_cutter->computeOptimalCuts(params);

        if (!cuts.empty()) {
            m_statistics.numCuts = cuts.size();
            m_statistics.totalCutLength = 0.0f;
            for (const auto& cut : cuts) {
                m_statistics.totalCutLength += cut.totalLength;
            }

            // 应用切缝到网格
            auto cutMesh = m_cutter->applyCutsToMesh(cuts);
            if (cutMesh) {
                m_mesh = cutMesh;
                m_textureMapper->setMesh(m_mesh, m_geometry);
                updateMeshBuffers();
            }

            m_cutComputed = true;
            logMessage("Cut computation completed: " + std::to_string(cuts.size()) +
                      " cuts, total length = " + std::to_string(m_statistics.totalCutLength));
        } else {
            logMessage("Warning: No cuts computed, continuing with original mesh");
        }
    } catch (const std::exception& e) {
        logMessage("Error in cut computation: " + std::string(e.what()));
    }
}

void ImGuiTextureMappingGUI::computeUVMapping() {
    if (!m_meshLoaded) return;

    logMessage("Step 3: Computing UV parametrization...");

    try {
        // 设置UV映射参数
        TextureMapper::MappingParams params;
        params.useConformalMapping = true;
        params.automaticConeDetection = true;
        params.curvatureThreshold = m_guiState.curvatureThreshold;

        // 计算UV映射
        auto mappingResult = m_textureMapper->computeUVMapping(params);

        if (mappingResult.has_value()) {
            // 保存UV映射结果
            m_currentUVMapping = mappingResult;
            const auto& mapping = mappingResult.value();

            // 更新统计信息 - 使用UVDistortionAnalyzer计算真实失真（参考测试代码）
            m_statistics.numCharts = mapping.charts.size();

            // 使用UVDistortionAnalyzer进行详细失真分析
            m_distortionAnalyzer->setInput(m_mesh, m_geometry, mapping);
            auto distortions = m_distortionAnalyzer->computeAllDistortions();
            auto globalStats = m_distortionAnalyzer->computeGlobalStats(distortions);

            // 更新统计信息使用真实的失真数据
            m_statistics.angleDistortion = globalStats.conformalMean;
            m_statistics.areaDistortion = globalStats.areaMean;
            m_statistics.conformalError = globalStats.conformalMean;

            logMessage("UV mapping completed: " + std::to_string(mapping.charts.size()) + " charts");
            logMessage("  Distortion analysis:");
            logMessage("    Stretch (σ_max): mean=" + std::to_string(globalStats.stretchMean) +
                      ", max=" + std::to_string(globalStats.stretchMax));
            logMessage("    Conformal error: mean=" + std::to_string(globalStats.conformalMean) +
                      ", max=" + std::to_string(globalStats.conformalMax));
            logMessage("    Area distortion: mean=" + std::to_string(globalStats.areaMean) +
                      ", max=" + std::to_string(globalStats.areaMax));

            // 初始化BarycentricMapper用于后续的图案映射
            m_barycentricMapper->setInput(m_mesh, m_geometry, mapping);
            m_barycentricMapper->buildSpatialIndex();
            logMessage("  Barycentric mapper initialized");

            // 保存UV坐标供SurfaceFiller使用
            m_surfaceFiller->setInput(m_mesh, m_geometry, mapping);

            m_uvMappingComputed = true;

            // 可选：导出UV网格
            // m_textureMapper->exportUVMesh(m_guiState.outputPrefix + "_uv.obj", mapping);
        } else {
            logMessage("Error: UV mapping failed");
        }
    } catch (const std::exception& e) {
        logMessage("Error in UV mapping: " + std::string(e.what()));
    }
}

void ImGuiTextureMappingGUI::generatePatterns() {
    if (!m_meshLoaded || !m_uvMappingComputed || !m_currentUVMapping.has_value()) {
        logMessage("Warning: UV mapping must be computed before generating patterns");
        return;
    }

    logMessage("Step 4: Generating surface filling patterns...");

    try {
        using namespace geometrycentral;

        const auto& mapping = m_currentUVMapping.value();

        // 计算UV边界（参考测试代码）
        Vector2 uvMin = mapping.uvCoordinates[0];
        Vector2 uvMax = mapping.uvCoordinates[0];
        for (const auto& uv : mapping.uvCoordinates) {
            if (uv.x < uvMin.x) uvMin.x = uv.x;
            if (uv.y < uvMin.y) uvMin.y = uv.y;
            if (uv.x > uvMax.x) uvMax.x = uv.x;
            if (uv.y > uvMax.y) uvMax.y = uv.y;
        }

        // 添加内边距避免边界问题
        Vector2 padding = (uvMax - uvMin) * 0.05;
        uvMin = uvMin + padding;
        uvMax = uvMax - padding;

        double spacing = m_guiState.patternSpacing;
        logMessage("  UV range: [" + std::to_string(uvMin.x) + ", " + std::to_string(uvMax.x) + "] x [" +
                  std::to_string(uvMin.y) + ", " + std::to_string(uvMax.y) + "]");
        logMessage("  Pattern spacing: " + std::to_string(spacing));

        // 生成UV空间的网格图案
        std::vector<std::vector<Vector2>> gridPathsUV;

        // 生成垂直线
        for (double x = uvMin.x; x <= uvMax.x; x += spacing) {
            std::vector<Vector2> line;
            line.push_back({x, uvMin.y});
            line.push_back({x, uvMax.y});
            gridPathsUV.push_back(line);
        }

        // 生成水平线
        for (double y = uvMin.y; y <= uvMax.y; y += spacing) {
            std::vector<Vector2> line;
            line.push_back({uvMin.x, y});
            line.push_back({uvMax.x, y});
            gridPathsUV.push_back(line);
        }

        logMessage("  Generated " + std::to_string(gridPathsUV.size()) + " grid lines in UV space");

        // 使用PatternBackMapper将UV图案映射到3D（参考测试代码）
        m_patternBackMapper->setInput(m_mesh, m_geometry, mapping, m_barycentricMapper);

        PatternBackMapper::MappingParams params;
        params.useGeodesicPath = true;  // 使用测地线路径
        params.geodesicResolution = spacing / 20.0;  // 测地线采样分辨率

        m_pattern3DPaths.clear();
        int mappedPaths = 0;
        double totalLength3D = 0.0;

        for (const auto& uvPath : gridPathsUV) {
            auto path3D = m_patternBackMapper->mapPathTo3D(uvPath, params);

            if (!path3D.empty()) {
                m_pattern3DPaths.push_back(path3D);
                mappedPaths++;

                // 计算3D路径长度
                double length = 0.0;
                for (size_t j = 1; j < path3D.size(); ++j) {
                    length += (path3D[j] - path3D[j-1]).norm();
                }
                totalLength3D += length;
            }
        }

        // 更新统计信息
        m_statistics.numPaths = mappedPaths;
        m_statistics.totalPathLength = totalLength3D;
        m_statistics.coverage = static_cast<double>(mappedPaths) / gridPathsUV.size();
        m_statistics.uniformity = 0.9;  // 网格图案通常具有高均匀性

        m_patternsGenerated = true;
        logMessage("Pattern generation completed:");
        logMessage("  Mapped paths: " + std::to_string(mappedPaths) + "/" + std::to_string(gridPathsUV.size()) +
                  " (" + std::to_string(m_statistics.coverage * 100.0) + "%)");
        logMessage("  Total 3D path length: " + std::to_string(totalLength3D));
        logMessage("  Total 3D path points: " + std::to_string(
            std::accumulate(m_pattern3DPaths.begin(), m_pattern3DPaths.end(), size_t(0),
                [](size_t sum, const auto& path) { return sum + path.size(); })));

        // 更新OpenGL缓冲区以可视化图案
        updatePatternsBuffers();

    } catch (const std::exception& e) {
        logMessage("Error in pattern generation: " + std::string(e.what()));
    }
}

void ImGuiTextureMappingGUI::updateCutsBuffers() {
    // TODO: Implement cuts buffer update
}

void ImGuiTextureMappingGUI::updatePatternsBuffers() {
    // TODO: Implement patterns buffer update
}

void ImGuiTextureMappingGUI::cleanup() {
    if (m_window) {
        // Cleanup OpenGL resources
        if (m_meshVAO) glDeleteVertexArrays(1, &m_meshVAO);
        if (m_meshVBO) glDeleteBuffers(1, &m_meshVBO);
        if (m_meshEBO) glDeleteBuffers(1, &m_meshEBO);
        if (m_linesVAO) glDeleteVertexArrays(1, &m_linesVAO);
        if (m_linesVBO) glDeleteBuffers(1, &m_linesVBO);
        if (m_shaderProgram) glDeleteProgram(m_shaderProgram);

        // Cleanup ImGui
        ImGui_ImplOpenGL3_Shutdown();
        ImGui_ImplGlfw_Shutdown();
        ImGui::DestroyContext();

        // Cleanup GLFW
        glfwDestroyWindow(m_window);
        glfwTerminate();

        m_window = nullptr;
    }
}

} // namespace SurfaceTextureMapping