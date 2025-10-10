/**
 * ImGui UV失真可视化查看器实现
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include "imgui_distortion_viewer.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"

namespace SurfaceTextureMapping {

// ============================================================================
// 构造和析构
// ============================================================================

ImGuiDistortionViewer::ImGuiDistortionViewer() {
    m_analyzer = std::make_unique<UVDistortionAnalyzer>();
}

ImGuiDistortionViewer::~ImGuiDistortionViewer() {
    cleanup();
}

// ============================================================================
// 初始化
// ============================================================================

bool ImGuiDistortionViewer::initialize() {
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

    if (!loadShaders()) {
        std::cerr << "Failed to load shaders" << std::endl;
        return false;
    }

    logMessage("Distortion Viewer initialized successfully");
    return true;
}

bool ImGuiDistortionViewer::initializeGLFW() {
    if (!glfwInit()) {
        return false;
    }

    // OpenGL 3.3 Core Profile
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_SAMPLES, 4); // MSAA

    m_window = glfwCreateWindow(m_windowWidth, m_windowHeight,
                                 "UV Distortion Viewer", nullptr, nullptr);
    if (!m_window) {
        glfwTerminate();
        return false;
    }

    glfwMakeContextCurrent(m_window);
    glfwSwapInterval(1); // VSync

    // 设置回调
    glfwSetWindowUserPointer(m_window, this);
    glfwSetMouseButtonCallback(m_window, mouseButtonCallback);
    glfwSetCursorPosCallback(m_window, cursorPosCallback);
    glfwSetScrollCallback(m_window, scrollCallback);
    glfwSetKeyCallback(m_window, keyCallback);

    return true;
}

bool ImGuiDistortionViewer::initializeOpenGL() {
    // 初始化GLEW
    glewExperimental = GL_TRUE;
    if (glewInit() != GLEW_OK) {
        return false;
    }

    // OpenGL状态设置
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_MULTISAMPLE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 清除颜色
    glClearColor(0.15f, 0.15f, 0.15f, 1.0f);

    return true;
}

bool ImGuiDistortionViewer::initializeImGui() {
    // 初始化ImGui上下文
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    // 启用键盘导航
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    // Docking特性在某些ImGui版本中可能不可用，需要ImGui v1.80+
    // io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    // 设置ImGui样式
    ImGui::StyleColorsDark();

    // 初始化ImGui后端
    ImGui_ImplGlfw_InitForOpenGL(m_window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    return true;
}

bool ImGuiDistortionViewer::loadShaders() {
    // 顶点着色器 (3D视图)
    const char* vertexShaderSource3D = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        layout (location = 1) in vec3 aColor;

        out vec3 vertexColor;

        uniform mat4 model;
        uniform mat4 view;
        uniform mat4 projection;

        void main() {
            gl_Position = projection * view * model * vec4(aPos, 1.0);
            vertexColor = aColor;
        }
    )";

    // 片段着色器 (3D视图)
    const char* fragmentShaderSource3D = R"(
        #version 330 core
        in vec3 vertexColor;
        out vec4 FragColor;

        void main() {
            FragColor = vec4(vertexColor, 1.0);
        }
    )";

    // 编译3D着色器
    GLuint vertexShader3D = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader3D, 1, &vertexShaderSource3D, nullptr);
    glCompileShader(vertexShader3D);

    // 检查编译错误
    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(vertexShader3D, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader3D, 512, nullptr, infoLog);
        std::cerr << "Vertex shader compilation failed: " << infoLog << std::endl;
        return false;
    }

    GLuint fragmentShader3D = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader3D, 1, &fragmentShaderSource3D, nullptr);
    glCompileShader(fragmentShader3D);

    glGetShaderiv(fragmentShader3D, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader3D, 512, nullptr, infoLog);
        std::cerr << "Fragment shader compilation failed: " << infoLog << std::endl;
        return false;
    }

    // 链接3D着色器程序
    m_shaderProgram3D = glCreateProgram();
    glAttachShader(m_shaderProgram3D, vertexShader3D);
    glAttachShader(m_shaderProgram3D, fragmentShader3D);
    glLinkProgram(m_shaderProgram3D);

    glGetProgramiv(m_shaderProgram3D, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(m_shaderProgram3D, 512, nullptr, infoLog);
        std::cerr << "Shader program linking failed: " << infoLog << std::endl;
        return false;
    }

    glDeleteShader(vertexShader3D);
    glDeleteShader(fragmentShader3D);

    // UV着色器使用相同的着色器代码
    m_shaderProgramUV = m_shaderProgram3D;

    return true;
}

// ============================================================================
// 主循环
// ============================================================================

void ImGuiDistortionViewer::run() {
    while (!glfwWindowShouldClose(m_window)) {
        glfwPollEvents();

        // 开始ImGui帧
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 绘制GUI
        drawMainMenuBar();
        drawControlPanel();
        drawViewportPanel();
        if (m_showStatsPanel) {
            drawStatisticsPanel();
        }

        // 渲染场景
        render();

        // 渲染ImGui
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(m_window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(m_window);
    }
}

// ============================================================================
// 渲染
// ============================================================================

void ImGuiDistortionViewer::render() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    if (!m_meshLoaded) {
        return;
    }

    // 左侧: 3D视图
    // 右侧: UV视图
    // (实际渲染将在viewport panel中使用FBO进行)
}

void ImGuiDistortionViewer::render3DView() {
    // TODO: 实现3D网格渲染
}

void ImGuiDistortionViewer::renderUVView() {
    // TODO: 实现UV空间渲染
}

void ImGuiDistortionViewer::renderColorLegend() {
    // TODO: 实现颜色图例
}

// ============================================================================
// GUI界面
// ============================================================================

void ImGuiDistortionViewer::drawMainMenuBar() {
    if (ImGui::BeginMainMenuBar()) {
        if (ImGui::BeginMenu("File")) {
            if (ImGui::MenuItem("Load Mesh with UV...")) {
                // TODO: 文件对话框
            }
            ImGui::Separator();
            if (ImGui::MenuItem("Exit")) {
                glfwSetWindowShouldClose(m_window, true);
            }
            ImGui::EndMenu();
        }

        if (ImGui::BeginMenu("View")) {
            ImGui::MenuItem("Control Panel", nullptr, &m_showControlPanel);
            ImGui::MenuItem("Statistics", nullptr, &m_showStatsPanel);
            ImGui::MenuItem("Log", nullptr, &m_showLogPanel);
            ImGui::EndMenu();
        }

        ImGui::EndMainMenuBar();
    }
}

void ImGuiDistortionViewer::drawControlPanel() {
    if (!m_showControlPanel) return;

    ImGui::Begin("Control Panel", &m_showControlPanel);

    if (ImGui::CollapsingHeader("Distortion Metric", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char* metrics[] = { "Stretch", "Compression", "Conformal", "Area Distortion" };
        int currentMetric = static_cast<int>(m_viewConfig.metric);
        if (ImGui::Combo("Metric", &currentMetric, metrics, IM_ARRAYSIZE(metrics))) {
            m_viewConfig.metric = static_cast<DistortionMetric>(currentMetric);
            updateDistortionColors();
        }
    }

    if (ImGui::CollapsingHeader("Color Map", ImGuiTreeNodeFlags_DefaultOpen)) {
        const char* colormaps[] = { "Viridis", "Plasma", "Inferno", "Jet", "Grayscale", "Turbo", "CoolWarm" };
        int currentColormap = static_cast<int>(m_viewConfig.colormap);
        if (ImGui::Combo("Colormap", &currentColormap, colormaps, IM_ARRAYSIZE(colormaps))) {
            m_viewConfig.colormap = static_cast<ColorMap>(currentColormap);
            updateDistortionColors();
        }

        ImGui::Checkbox("Auto Range", &m_viewConfig.autoRange);
        if (!m_viewConfig.autoRange) {
            ImGui::SliderFloat("Min", &m_viewConfig.minDistortion, 0.0f, 5.0f);
            ImGui::SliderFloat("Max", &m_viewConfig.maxDistortion, 1.0f, 20.0f);
        }
    }

    if (ImGui::CollapsingHeader("Visualization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Show Wireframe", &m_viewConfig.showWireframe);
        ImGui::Checkbox("Show Distortion", &m_viewConfig.showDistortion);
        ImGui::Checkbox("Show UV Layout", &m_viewConfig.showUVLayout);
        ImGui::Checkbox("Highlight High Distortion", &m_viewConfig.showHighDistortionRegions);

        if (m_viewConfig.showHighDistortionRegions) {
            ImGui::SliderFloat("Threshold", &m_viewConfig.distortionThreshold, 1.0f, 10.0f);
        }
    }

    if (ImGui::Button("Analyze Distortion", ImVec2(-1, 0))) {
        analyzeDistortion();
    }

    ImGui::End();
}

void ImGuiDistortionViewer::drawViewportPanel() {
    ImGui::Begin("Viewport", nullptr,
                 ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);

    ImVec2 viewportSize = ImGui::GetContentRegionAvail();

    // 左右分屏
    ImGui::BeginChild("3D View", ImVec2(viewportSize.x * m_viewportSplitRatio, viewportSize.y), true);
    ImGui::Text("3D View");
    // TODO: 渲染3D视图到纹理，然后显示
    ImGui::EndChild();

    ImGui::SameLine();

    ImGui::BeginChild("UV View", ImVec2(0, viewportSize.y), true);
    ImGui::Text("UV View");
    // TODO: 渲染UV视图到纹理，然后显示
    ImGui::EndChild();

    ImGui::End();
}

void ImGuiDistortionViewer::drawStatisticsPanel() {
    ImGui::Begin("Statistics", &m_showStatsPanel);

    if (m_distortionComputed) {
        ImGui::Text("Mesh Info:");
        ImGui::Separator();
        ImGui::Text("Vertices: %d", m_stats.numVertices);
        ImGui::Text("Faces: %d", m_stats.numFaces);

        ImGui::Spacing();
        ImGui::Text("Distortion Statistics:");
        ImGui::Separator();
        ImGui::Text("Avg Stretch: %.3f", m_stats.avgStretch);
        ImGui::Text("Max Stretch: %.3f", m_stats.maxStretch);
        ImGui::Text("Avg Conformal: %.3f", m_stats.avgConformal);
        ImGui::Text("Max Conformal: %.3f", m_stats.maxConformal);
        ImGui::Text("Avg Area Ratio: %.3f", m_stats.avgArea);

        ImGui::Spacing();
        ImGui::Text("Quality:");
        ImGui::Separator();
        ImGui::Text("High Distortion Faces: %d", m_stats.numHighDistortionFaces);

        float passRate = 100.0f * (m_stats.numFaces - m_stats.numHighDistortionFaces) /
                         static_cast<float>(m_stats.numFaces);
        ImGui::Text("Pass Rate: %.1f%%", passRate);
    } else {
        ImGui::TextDisabled("No distortion data available");
        ImGui::Text("Load a mesh with UV coordinates and click 'Analyze Distortion'");
    }

    ImGui::End();
}

void ImGuiDistortionViewer::drawColorMapLegend() {
    // TODO: 绘制颜色图例条
}

// ============================================================================
// 数据设置
// ============================================================================

void ImGuiDistortionViewer::setMeshData(
    std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh,
    std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
    const TextureMapper::UVMapping& uvMapping)
{
    m_mesh = mesh;
    m_geometry = geometry;
    m_uvMapping = uvMapping;

    m_meshLoaded = true;

    // 设置分析器输入
    m_analyzer->setInput(m_mesh, m_geometry, m_uvMapping);

    // 更新渲染缓冲区
    updateMeshBuffers();
    updateUVMeshBuffers();

    // 计算统计
    m_stats.numVertices = m_mesh->nVertices();
    m_stats.numFaces = m_mesh->nFaces();

    logMessage("Mesh data loaded: " + std::to_string(m_stats.numVertices) + " vertices, " +
               std::to_string(m_stats.numFaces) + " faces");
}

bool ImGuiDistortionViewer::loadMeshWithUV(const std::string& filename) {
    // TODO: 实现OBJ文件加载（包含UV坐标）
    logMessage("Loading mesh: " + filename);
    return false;
}

// ============================================================================
// 失真分析
// ============================================================================

void ImGuiDistortionViewer::analyzeDistortion() {
    if (!m_meshLoaded) {
        logMessage("Error: No mesh loaded");
        return;
    }

    logMessage("Analyzing distortion...");

    // 计算失真
    m_distortions = m_analyzer->computeAllDistortions();
    m_globalStats = m_analyzer->computeGlobalStats(m_distortions);

    // 标记高失真区域
    m_highDistortionFaces = m_analyzer->markHighDistortionRegions(
        m_distortions, m_viewConfig.distortionThreshold);

    m_distortionComputed = true;

    // 更新统计信息
    m_stats.avgStretch = m_globalStats.stretchMean;
    m_stats.maxStretch = m_globalStats.stretchMax;
    m_stats.avgConformal = m_globalStats.conformalMean;
    m_stats.maxConformal = m_globalStats.conformalMax;
    m_stats.avgArea = m_globalStats.areaMean;
    m_stats.numHighDistortionFaces = m_highDistortionFaces.size();

    // 更新颜色
    updateDistortionColors();

    logMessage("Distortion analysis complete");
}

void ImGuiDistortionViewer::computeDistortionStatistics() {
    // 已在analyzeDistortion()中完成
}

glm::vec3 ImGuiDistortionViewer::getDistortionColor(float distortionValue) {
    // 归一化到[0, 1]
    float t;
    if (m_viewConfig.autoRange) {
        float minVal = 1.0f; // 完美映射
        float maxVal = m_globalStats.stretchMax;
        t = (distortionValue - minVal) / (maxVal - minVal + 1e-6f);
    } else {
        t = (distortionValue - m_viewConfig.minDistortion) /
            (m_viewConfig.maxDistortion - m_viewConfig.minDistortion + 1e-6f);
    }

    t = std::clamp(t, 0.0f, 1.0f);

    return evaluateColorMap(t, m_viewConfig.colormap);
}

// ============================================================================
// 颜色映射
// ============================================================================

glm::vec3 ImGuiDistortionViewer::evaluateColorMap(float t, ColorMap colormap) {
    t = std::clamp(t, 0.0f, 1.0f);

    switch (colormap) {
        case ColorMap::VIRIDIS:
            // 简化版Viridis (紫绿渐变)
            if (t < 0.5f) {
                float s = t * 2.0f;
                return glm::mix(glm::vec3(0.267f, 0.004f, 0.329f),
                               glm::vec3(0.282f, 0.518f, 0.545f), s);
            } else {
                float s = (t - 0.5f) * 2.0f;
                return glm::mix(glm::vec3(0.282f, 0.518f, 0.545f),
                               glm::vec3(0.993f, 0.906f, 0.144f), s);
            }

        case ColorMap::JET:
            // 经典Jet: 蓝→青→绿→黄→红
            if (t < 0.25f) {
                return glm::mix(glm::vec3(0.0f, 0.0f, 0.5f), glm::vec3(0.0f, 0.5f, 1.0f), t * 4.0f);
            } else if (t < 0.5f) {
                return glm::mix(glm::vec3(0.0f, 0.5f, 1.0f), glm::vec3(0.0f, 1.0f, 0.0f), (t - 0.25f) * 4.0f);
            } else if (t < 0.75f) {
                return glm::mix(glm::vec3(0.0f, 1.0f, 0.0f), glm::vec3(1.0f, 1.0f, 0.0f), (t - 0.5f) * 4.0f);
            } else {
                return glm::mix(glm::vec3(1.0f, 1.0f, 0.0f), glm::vec3(1.0f, 0.0f, 0.0f), (t - 0.75f) * 4.0f);
            }

        case ColorMap::GRAYSCALE:
            return glm::vec3(t, t, t);

        case ColorMap::COOLWARM:
            return glm::mix(glm::vec3(0.23f, 0.30f, 0.75f),
                           glm::vec3(0.706f, 0.016f, 0.150f), t);

        default:
            return glm::vec3(t, 0.0f, 1.0f - t);
    }
}

// ============================================================================
// 缓冲区更新
// ============================================================================

void ImGuiDistortionViewer::updateMeshBuffers() {
    // TODO: 从geometry-central网格提取顶点和面片数据
    // 创建OpenGL缓冲区
}

void ImGuiDistortionViewer::updateDistortionColors() {
    if (!m_distortionComputed) {
        return;
    }

    // TODO: 根据失真值计算顶点颜色
}

void ImGuiDistortionViewer::updateUVMeshBuffers() {
    // TODO: 从UV坐标创建UV空间网格
}

// ============================================================================
// 相机控制
// ============================================================================

void ImGuiDistortionViewer::updateCamera3D() {
    // 球面坐标相机
    float camX = m_camera3D.distance * cos(m_camera3D.rotationY) * cos(m_camera3D.rotationX);
    float camY = m_camera3D.distance * sin(m_camera3D.rotationY);
    float camZ = m_camera3D.distance * cos(m_camera3D.rotationY) * sin(m_camera3D.rotationX);

    m_camera3D.position = glm::vec3(camX, camY, camZ) + m_camera3D.target;
}

void ImGuiDistortionViewer::updateCameraUV() {
    // 正交投影，简单的平移和缩放
}

void ImGuiDistortionViewer::resetCamera() {
    m_camera3D = Camera3D();
    m_cameraUV = CameraUV();
}

// ============================================================================
// 事件回调
// ============================================================================

void ImGuiDistortionViewer::mouseButtonCallback(GLFWwindow* window, int button, int action, int mods) {
    ImGuiDistortionViewer* viewer = static_cast<ImGuiDistortionViewer*>(glfwGetWindowUserPointer(window));

    if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
        viewer->m_mousePressed = true;
        glfwGetCursorPos(window, &viewer->m_lastMouseX, &viewer->m_lastMouseY);
    } else if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_RELEASE) {
        viewer->m_mousePressed = false;
    }

    if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_PRESS) {
        viewer->m_middleMousePressed = true;
        glfwGetCursorPos(window, &viewer->m_lastMouseX, &viewer->m_lastMouseY);
    } else if (button == GLFW_MOUSE_BUTTON_MIDDLE && action == GLFW_RELEASE) {
        viewer->m_middleMousePressed = false;
    }
}

void ImGuiDistortionViewer::cursorPosCallback(GLFWwindow* window, double xpos, double ypos) {
    ImGuiDistortionViewer* viewer = static_cast<ImGuiDistortionViewer*>(glfwGetWindowUserPointer(window));

    if (viewer->m_mousePressed) {
        double dx = xpos - viewer->m_lastMouseX;
        double dy = ypos - viewer->m_lastMouseY;

        viewer->m_camera3D.rotationX += dx * 0.01f;
        viewer->m_camera3D.rotationY += dy * 0.01f;

        // 限制Y旋转
        viewer->m_camera3D.rotationY = std::clamp(viewer->m_camera3D.rotationY,
                                                   -static_cast<float>(M_PI) * 0.49f,
                                                   static_cast<float>(M_PI) * 0.49f);

        viewer->updateCamera3D();
    }

    viewer->m_lastMouseX = xpos;
    viewer->m_lastMouseY = ypos;
}

void ImGuiDistortionViewer::scrollCallback(GLFWwindow* window, double xoffset, double yoffset) {
    ImGuiDistortionViewer* viewer = static_cast<ImGuiDistortionViewer*>(glfwGetWindowUserPointer(window));

    viewer->m_camera3D.distance *= (1.0f - yoffset * 0.1f);
    viewer->m_camera3D.distance = std::clamp(viewer->m_camera3D.distance, 0.5f, 10.0f);

    viewer->updateCamera3D();
}

void ImGuiDistortionViewer::keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    ImGuiDistortionViewer* viewer = static_cast<ImGuiDistortionViewer*>(glfwGetWindowUserPointer(window));

    if (key == GLFW_KEY_R && action == GLFW_PRESS) {
        viewer->resetCamera();
    }
}

// ============================================================================
// 工具函数
// ============================================================================

void ImGuiDistortionViewer::logMessage(const std::string& message) {
    m_logMessages.push_back(message);
    std::cout << "[DistortionViewer] " << message << std::endl;
}

// ============================================================================
// 清理
// ============================================================================

void ImGuiDistortionViewer::cleanup() {
    // 清理OpenGL对象
    if (m_meshVAO) glDeleteVertexArrays(1, &m_meshVAO);
    if (m_meshVBO) glDeleteBuffers(1, &m_meshVBO);
    if (m_meshEBO) glDeleteBuffers(1, &m_meshEBO);
    if (m_colorVBO) glDeleteBuffers(1, &m_colorVBO);

    if (m_uvVAO) glDeleteVertexArrays(1, &m_uvVAO);
    if (m_uvVBO) glDeleteBuffers(1, &m_uvVBO);
    if (m_uvEBO) glDeleteBuffers(1, &m_uvEBO);
    if (m_uvColorVBO) glDeleteBuffers(1, &m_uvColorVBO);

    if (m_shaderProgram3D) glDeleteProgram(m_shaderProgram3D);

    // 清理ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // 清理GLFW
    if (m_window) {
        glfwDestroyWindow(m_window);
    }
    glfwTerminate();
}

} // namespace SurfaceTextureMapping