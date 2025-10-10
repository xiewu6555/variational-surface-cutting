#pragma once

/**
 * ImGui UV失真可视化查看器
 *
 * 功能特性:
 * - 双视图布局 (3D网格视图 + UV空间视图)
 * - 失真热力图可视化 (拉伸失真, 共形误差, 面积失真)
 * - 交互式失真分析
 * - 颜色映射控制
 * - 统计信息显示
 * - 高失真区域标记
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <optional>

// OpenGL related (GLEW must be before other OpenGL headers)
#include <GL/glew.h>
#include <GLFW/glfw3.h>

// ImGui related (after OpenGL)
#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

// GLM for math calculations
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

// Geometry Central
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

// Surface Texture Mapping core
#include "uv_distortion_analyzer.h"
#include "texture_mapping.h"

namespace SurfaceTextureMapping {

/**
 * 失真可视化查看器 - ImGui版本
 *
 * 提供专业的UV失真分析和可视化界面
 */
class ImGuiDistortionViewer {
public:
    /**
     * 颜色映射类型
     */
    enum class ColorMap {
        VIRIDIS,    // 紫绿渐变 (推荐, 色盲友好)
        PLASMA,     // 紫粉渐变
        INFERNO,    // 黑紫红黄渐变
        JET,        // 蓝绿黄红渐变 (经典但不推荐)
        GRAYSCALE,  // 灰度
        TURBO,      // Google Turbo (感知均匀)
        COOLWARM    // 冷暖对比
    };

    /**
     * 失真度量类型
     */
    enum class DistortionMetric {
        STRETCH,        // 拉伸失真 (σ_max)
        COMPRESSION,    // 压缩失真 (1/σ_min)
        CONFORMAL,      // 共形误差 (QC = σ_max/σ_min)
        AREA_DISTORTION // 面积失真
    };

    /**
     * 视图配置
     */
    struct ViewConfig {
        // 可见性控制
        bool showWireframe = true;
        bool showDistortion = true;
        bool showUVLayout = true;
        bool showHighDistortionRegions = false;

        // 渲染参数
        float wireframeWidth = 1.0f;
        ColorMap colormap = ColorMap::VIRIDIS;
        DistortionMetric metric = DistortionMetric::CONFORMAL;

        // 失真阈值
        float distortionThreshold = 2.0f;
        bool usePercentile = true;
        float percentileThreshold = 95.0f;

        // 颜色映射范围控制
        bool autoRange = true;
        float minDistortion = 0.0f;
        float maxDistortion = 10.0f;
    };

public:
    ImGuiDistortionViewer();
    ~ImGuiDistortionViewer();

    /**
     * 初始化查看器
     */
    bool initialize();

    /**
     * 运行主循环
     */
    void run();

    /**
     * 设置网格和UV映射数据
     */
    void setMeshData(
        std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh,
        std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> geometry,
        const TextureMapper::UVMapping& uvMapping
    );

    /**
     * 加载网格文件 (OBJ格式, 需要包含UV坐标)
     */
    bool loadMeshWithUV(const std::string& filename);

    /**
     * 清理资源
     */
    void cleanup();

private:
    // ========================================================================
    // GLFW和OpenGL初始化
    // ========================================================================
    bool initializeGLFW();
    bool initializeImGui();
    bool initializeOpenGL();
    bool loadShaders();

    // ========================================================================
    // 渲染系统
    // ========================================================================
    void render();
    void render3DView();
    void renderUVView();
    void renderColorLegend();

    // OpenGL缓冲区管理
    void updateMeshBuffers();
    void updateDistortionColors();
    void updateUVMeshBuffers();

    // ========================================================================
    // 失真分析
    // ========================================================================
    void analyzeDistortion();
    void computeDistortionStatistics();
    glm::vec3 getDistortionColor(float distortionValue);

    // ========================================================================
    // GUI界面
    // ========================================================================
    void drawMainMenuBar();
    void drawControlPanel();
    void drawViewportPanel();
    void drawStatisticsPanel();
    void drawColorMapLegend();

    // ========================================================================
    // 相机控制
    // ========================================================================
    void updateCamera3D();
    void updateCameraUV();
    void resetCamera();

    // 鼠标和键盘事件
    static void mouseButtonCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorPosCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    // ========================================================================
    // 工具函数
    // ========================================================================
    void logMessage(const std::string& message);
    glm::vec3 evaluateColorMap(float t, ColorMap colormap);

private:
    // ========================================================================
    // 窗口和OpenGL状态
    // ========================================================================
    GLFWwindow* m_window = nullptr;
    int m_windowWidth = 1920;
    int m_windowHeight = 1080;

    // OpenGL对象
    GLuint m_shaderProgram3D = 0;
    GLuint m_shaderProgramUV = 0;

    // 3D视图渲染对象
    GLuint m_meshVAO = 0;
    GLuint m_meshVBO = 0;
    GLuint m_meshEBO = 0;
    GLuint m_colorVBO = 0;  // 失真颜色

    // UV视图渲染对象
    GLuint m_uvVAO = 0;
    GLuint m_uvVBO = 0;
    GLuint m_uvEBO = 0;
    GLuint m_uvColorVBO = 0;

    // ========================================================================
    // 相机状态
    // ========================================================================
    struct Camera3D {
        glm::vec3 position = glm::vec3(0.0f, 0.0f, 3.0f);
        glm::vec3 target = glm::vec3(0.0f, 0.0f, 0.0f);
        glm::vec3 up = glm::vec3(0.0f, 1.0f, 0.0f);
        float distance = 3.0f;
        float rotationX = 0.0f;
        float rotationY = 0.0f;
        float fov = 45.0f;
    } m_camera3D;

    struct CameraUV {
        glm::vec2 center = glm::vec2(0.5f, 0.5f);
        float zoom = 1.0f;
    } m_cameraUV;

    // 鼠标交互状态
    bool m_mousePressed = false;
    bool m_middleMousePressed = false;
    double m_lastMouseX = 0.0;
    double m_lastMouseY = 0.0;

    // ========================================================================
    // 网格数据
    // ========================================================================
    std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh> m_mesh;
    std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> m_geometry;
    TextureMapper::UVMapping m_uvMapping;

    // 渲染数据
    std::vector<float> m_meshVertices3D;     // 3D顶点位置
    std::vector<unsigned int> m_meshIndices; // 面片索引
    std::vector<float> m_meshColors;         // 失真颜色 (per-vertex)

    std::vector<float> m_meshVerticesUV;     // UV空间顶点

    // ========================================================================
    // 失真分析数据
    // ========================================================================
    std::unique_ptr<UVDistortionAnalyzer> m_analyzer;
    std::vector<UVDistortionAnalyzer::FaceDistortion> m_distortions;
    UVDistortionAnalyzer::GlobalDistortionStats m_globalStats;
    std::vector<size_t> m_highDistortionFaces;

    // ========================================================================
    // GUI状态
    // ========================================================================
    ViewConfig m_viewConfig;

    bool m_meshLoaded = false;
    bool m_distortionComputed = false;

    // 面板可见性
    bool m_showControlPanel = true;
    bool m_showStatsPanel = true;
    bool m_showLogPanel = false;

    // 视口分割 (左右分屏)
    float m_viewportSplitRatio = 0.5f;

    // 统计信息显示
    struct Statistics {
        int numVertices = 0;
        int numFaces = 0;
        float avgStretch = 0.0f;
        float maxStretch = 0.0f;
        float avgConformal = 0.0f;
        float maxConformal = 0.0f;
        float avgArea = 0.0f;
        int numHighDistortionFaces = 0;
    } m_stats;

    // 日志系统
    std::vector<std::string> m_logMessages;
    bool m_autoScroll = true;

    // 状态消息
    std::string m_statusMessage = "Ready";
};

} // namespace SurfaceTextureMapping