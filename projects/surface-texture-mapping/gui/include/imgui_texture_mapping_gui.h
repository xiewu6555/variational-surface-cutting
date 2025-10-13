#pragma once

#include <memory>
#include <string>
#include <vector>
#include <array>
#include <map>
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

// Include geometry-central headers for declarations
#include <geometrycentral/surface/halfedge_mesh.h>
#include <geometrycentral/surface/vertex_position_geometry.h>

// Include texture mapping header for UVMapping type (required for std::optional)
#include "../../core/include/texture_mapping.h"

// Forward declarations for algorithm classes
namespace SurfaceTextureMapping {
    class AlgorithmPipeline;
    class VariationalCuttingAlgorithm;
    class UVUnwrappingAlgorithm;
    class MeshProcessor;
    class VariationalCutter;
    class SurfaceFiller;
    class UVDistortionAnalyzer;
    class BarycentricMapper;
    class PatternBackMapper;
}

// Forward declare CGAL types instead of including header
namespace SurfaceTextureMapping {
    // 简化的网格类型（用于代替CGAL Surface_mesh）
    struct SurfaceMesh {
        std::vector<std::array<float, 3>> vertices;
        std::vector<std::array<int, 3>> faces;
    };

    // 网格质量统计结构
    struct MeshQualityStats {
        double min_edge_length = 0.0;
        double max_edge_length = 0.0;
        double avg_edge_length = 0.0;
        double edge_length_std_dev = 0.0;
        int degenerate_faces = 0;
        int self_intersections = 0;
        bool is_manifold = false;
        bool is_closed = false;
        bool is_oriented = false;
        double surface_area = 0.0;
        double volume = 0.0;
        int genus = 0;
        int boundary_components = 0;
    };
}

// Use namespace to avoid conflicts with core Vector3
namespace STM {
    struct Vector3 {
        double x, y, z;
        Vector3() : x(0), y(0), z(0) {}
        Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

        // Basic operators
        Vector3 operator-(const Vector3& other) const {
            return Vector3(x - other.x, y - other.y, z - other.z);
        }
        Vector3 operator+(const Vector3& other) const {
            return Vector3(x + other.x, y + other.y, z + other.z);
        }
        Vector3 operator*(double scalar) const {
            return Vector3(x * scalar, y * scalar, z * scalar);
        }
    };
}

// Forward declarations for algorithm interfaces
class EulerianShapeOptimizer;
class BoundaryFirstFlattening;

// Simplified algorithm interfaces (temporary solution)
class SimpleShapeOptimizer {
public:
    SimpleShapeOptimizer(geometrycentral::surface::VertexPositionGeometry* geometry) : m_geometry(geometry) {}

    // Parameters
    double weightLengthRegularization = 1.0;
    double weightBilapRegularization = 1.0;

    void initializeData() { /* simplified */ }
    void initializeTimestep() { /* simplified */ }
    void doStep() { /* simplified */ }
    double computeEnergy() { return 0.5; /* placeholder */ }
    double evaluateEnergyTermLengthRegularization() { return 0.15; /* placeholder */ }

    // Return synthetic boundary lines for demonstration
    std::vector<std::array<STM::Vector3, 2>> getBoundaryLines() {
        std::vector<std::array<STM::Vector3, 2>> lines;
        // Generate 3 synthetic tennis ball seam lines
        for (int i = 0; i < 3; i++) {
            STM::Vector3 start(-0.5 + i * 0.5, -1.0, 0.0);
            STM::Vector3 end(-0.5 + i * 0.5, 1.0, 0.0);
            std::array<STM::Vector3, 2> line;
            line[0] = start;
            line[1] = end;
            lines.push_back(line);
        }
        return lines;
    }

private:
    geometrycentral::surface::VertexPositionGeometry* m_geometry;
};

class SimpleFlattener {
public:
    SimpleFlattener(geometrycentral::surface::VertexPositionGeometry* geometry) : m_geometry(geometry) {}

    bool flatten() {
        // Simplified: generate synthetic UV coordinates for demonstration
        return true;
    }

private:
    geometrycentral::surface::VertexPositionGeometry* m_geometry;
};

namespace SurfaceTextureMapping {

/**
 * Independent texture mapping GUI based on ImGui
 * Provides 3D visualization and parameter adjustment interface
 */
class ImGuiTextureMappingGUI {
public:
    ImGuiTextureMappingGUI();
    ~ImGuiTextureMappingGUI();

    /**
     * Initialize GUI system
     */
    bool initialize();

    /**
     * Run GUI main loop
     */
    void run();

    /**
     * Load mesh from file (public interface for command-line loading)
     */
    bool loadMesh(const std::string& filename);

    /**
     * Cleanup resources
     */
    void cleanup();

private:
    // GLFW window management
    GLFWwindow* m_window;
    int m_windowWidth = 1600;
    int m_windowHeight = 900;

    // OpenGL渲染状态
    GLuint m_shaderProgram;
    GLuint m_meshVAO, m_meshVBO, m_meshEBO;
    GLuint m_linesVAO, m_linesVBO;
    GLuint m_uvGridVAO, m_uvGridVBO;
    GLuint m_patternVAO, m_patternVBO;

    // 相机控制
    glm::mat4 m_viewMatrix;
    glm::mat4 m_projectionMatrix;
    glm::vec3 m_cameraPosition = glm::vec3(0, 0, 3);
    glm::vec3 m_cameraTarget = glm::vec3(0, 0, 0);
    glm::vec3 m_cameraUp = glm::vec3(0, 1, 0);

    float m_cameraDistance = 3.0f;
    float m_cameraRotationX = 0.0f;
    float m_cameraRotationY = 0.0f;
    bool m_mousePressed = false;
    double m_lastMouseX = 0, m_lastMouseY = 0;

    // Surface Texture Mapping core components (真实算法)
    std::unique_ptr<MeshProcessor> m_meshProcessor;
    std::unique_ptr<VariationalCutter> m_cutter;
    std::unique_ptr<TextureMapper> m_textureMapper;
    std::unique_ptr<SurfaceFiller> m_surfaceFiller;
    std::unique_ptr<UVDistortionAnalyzer> m_distortionAnalyzer;
    std::shared_ptr<BarycentricMapper> m_barycentricMapper;  // shared_ptr for PatternBackMapper
    std::unique_ptr<PatternBackMapper> m_patternBackMapper;

    // 网格数据（使用真实类型）
    std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh> m_mesh;
    std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> m_geometry;

    // 渲染数据
    std::vector<float> m_meshVertices;
    std::vector<unsigned int> m_meshIndices;
    std::vector<float> m_lineVertices;      // Cut lines
    std::vector<float> m_uvGridVertices;    // UV grid lines
    std::vector<float> m_patternVertices;   // Pattern lines

    // 存储切割曲线（用于渲染）
    std::vector<VariationalCutter::CutCurve> m_cutCurves;

    // GUI状态
    struct GUIState {
        // 文件操作
        char inputFile[512] = "data/spot.obj";
        char outputPrefix[256] = "output";

        // 网格处理参数
        // 重要：Variational Surface Cutting算法需要平衡网格质量和计算性能
        //
        // ⭐⭐⭐ 性能约束（基于ANALYSIS_no_interior_vertices_deep_dive.md深度分析 - 2025-10-13）：
        //   ┌─────────────────────────────────────────────────────────────────┐
        //   │ 最小顶点数：5000（避免"no interior vertices"退化patches错误）│
        //   │ 理想顶点数：5000-8000（最优性能和质量平衡）                │
        //   │ 硬限制顶点数：12000（超过直接失败，避免内存溢出）         │
        //   └─────────────────────────────────────────────────────────────────┘
        //
        // ⭐⭐⭐ 实测数据（Spot模型，对角线长度~2.4，基于深度分析）：
        //   Target Edge Length    顶点数      状态       说明
        //   ──────────────────────────────────────────────────────────────
        //   0.010                 ~30k        ❌ 失败    超过12k硬限制 → 构建失败
        //   0.012                 ~20k        ❌ 失败    超过12k硬限制
        //   0.015                 ~14k        ⚠️  超限    接近12k → 可能失败
        //   0.018                 ~10k        ⚠️  边缘    接近上限
        //   0.020                 ~8-9k       ✅ 理想    最佳平衡点（强烈推荐）
        //   0.025                 ~6-7k       ✅ 良好    稳定可靠
        //   0.030                 ~5-6k       ✅ 可用    最小可靠配置（默认值）
        //   0.035                 ~3-4k       ⚠️  偏低    可能触发5k下限检查
        //   0.040                 ~2-3k       ❌ 失败    < 5k最小值 → "resolution too low"错误
        //
        // ⭐⭐⭐ 深度分析结论（2025-10-13）：
        //   问题根源：Normal Clustering在低分辨率网格上产生拓扑退化的patches
        //   退化表现：某些patches（如尖端区域）所有顶点都在边界上（nInterior=0）
        //   数学约束：Yamabe方程求解器需要每个patch至少50个内部顶点
        //   不均匀分布：即使总顶点数3000，尖端patches可能只分到24个顶点（全是边界）
        //   安全策略：MIN_TOTAL_VERTICES=5000，应对最坏情况的patch分布
        //
        // ⭐⭐⭐ 推荐默认值（2025-10-13修订 v3）：
        //   Target Edge Length = 0.025 → 中间值策略（~7-8k顶点，最佳平衡）
        //   理由：
        //   - 0.020对部分模型产生18k+顶点（超过12k限制） ❌
        //   - 0.030产生~5-6k顶点（刚好达标，无安全边际） ⚠️
        //   - 0.025产生~7-8k顶点（处于5k-12k窗口中心，最安全） ✅
        //   - 为不同尺寸和复杂度的模型提供最大兼容性
        //   - 对于大多数模型（对角线0.5-5单位）都能稳定工作
        bool enableRemeshing = true;
        float targetEdgeLength = 0.025f;   // ⭐ 最终修订为0.025（v1:0.030→v2:0.020→v3:0.025，中间值策略）
        int remeshIterations = 10;         // 提升迭代次数（from 5→10，提高重网格化质量）
        bool protectBoundary = true;

        // 变分切缝参数
        bool enableCutting = true;
        float lengthRegularization = 0.1f;
        float smoothRegularization = 0.05f;
        int maxCuttingIterations = 300;  // GitHub README推荐值

        // 纹理映射参数
        bool enableTexturing = true;
        float curvatureThreshold = 0.15f;

        // 表面填充参数
        bool enableFilling = true;
        int patternType = 0; // 0=grid, 1=hex, 2=spiral, 3=hilbert
        float patternSpacing = 0.02f;
        int hilbertOrder = 3;

        // 可视化选项
        bool showWireframe = true;
        bool showFilled = true;
        bool showCuts = true;
        bool showUVMapping = false;
        bool showPatterns = true;

        // 渲染选项
        glm::vec3 meshColor = glm::vec3(0.7f, 0.7f, 0.9f);
        glm::vec3 wireframeColor = glm::vec3(0.2f, 0.2f, 0.2f);
        glm::vec3 cutColor = glm::vec3(1.0f, 0.2f, 0.2f);
        glm::vec3 patternColor = glm::vec3(0.1f, 0.8f, 0.1f);
    } m_guiState;

    // Processing pipeline state
    bool m_meshLoaded = false;
    bool m_processingInProgress = false;
    bool m_meshProcessed = false;
    bool m_cutComputed = false;
    bool m_uvMappingComputed = false;
    bool m_patternsGenerated = false;
    std::string m_statusMessage = "Ready";

    // Pipeline results storage (真实算法结果)
    std::unique_ptr<geometrycentral::surface::HalfedgeMesh> m_processedMesh;
    std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> m_processedGeometry;

    // 存储完整的UV映射结果（来自TextureMapper）
    std::optional<TextureMapper::UVMapping> m_currentUVMapping;

    // 存储生成的3D图案路径（用于渲染）
    std::vector<std::vector<geometrycentral::Vector3>> m_pattern3DPaths;

    // 统计信息
    struct Statistics {
        int numVertices = 0;
        int numFaces = 0;
        int numEdges = 0;
        float avgEdgeLength = 0.0f;
        float minEdgeLength = 0.0f;
        float maxEdgeLength = 0.0f;
        bool isManifold = false;
        bool isClosed = false;

        // 切缝统计
        int numCuts = 0;
        float totalCutLength = 0.0f;
        float avgDistortion = 0.0f;

        // UV映射统计
        int numCharts = 0;
        float angleDistortion = 0.0f;
        float areaDistortion = 0.0f;
        float conformalError = 0.0f;

        // 填充统计
        int numPaths = 0;
        float totalPathLength = 0.0f;
        float coverage = 0.0f;
        float uniformity = 0.0f;
    } m_statistics;

    // 初始化方法
    bool initializeGLFW();
    bool initializeImGui();
    bool initializeOpenGL();
    bool loadShaders();

    // 渲染方法
    void render();
    void renderMesh();
    void renderCuts();
    void renderUVGrid();
    void renderPatterns();

    // GUI面板
    void drawMainMenuBar();
    void drawControlPanel();
    void drawVisualizationPanel();
    void drawStatisticsPanel();
    void drawLogPanel();

    // 网格操作
    std::string openFileDialog();
    bool loadMeshFromFile(const std::string& filename);
    bool loadOBJFile(const std::string& filename);
    bool loadMeshGeometryFromFile(const std::string& filename);  // Load using geometry-central
    void updateMeshBuffers();
    void updateCutsBuffers();
    void updateUVGridBuffers();
    void updatePatternsBuffers();
    void updateMeshVisualization();
    void generateSyntheticCutLines();
    void generateUVGrid();
    void generatePatternLines();

    // Algorithm result visualization
    void extractCutLinesFromOptimizer();
    void extractUVGridFromFlattening();
    void convertMeshToRenderData();

    // CGAL mesh processing methods
    bool convertRenderDataToCGALMesh(SurfaceMesh& cgal_mesh);
    bool convertCGALMeshToRenderData(const SurfaceMesh& cgal_mesh);
    MeshQualityStats analyzeMeshQuality(const SurfaceMesh& cgal_mesh);
    bool repairMesh(SurfaceMesh& cgal_mesh);
    bool ensureProperOrientation(SurfaceMesh& cgal_mesh);
    bool removeSelfIntersections(SurfaceMesh& cgal_mesh);
    bool performIsotropicRemeshing(SurfaceMesh& cgal_mesh);
    void logInitialMeshStatistics(const MeshQualityStats& stats);
    void logFinalMeshStatistics(const MeshQualityStats& initial, const MeshQualityStats& final);
    void recomputeNormals();

    // 处理流水线
    void runFullPipeline();
    void processMesh();
    void computeCuts();
    void computeUVMapping();
    void generatePatterns();

    // 网格质量检查辅助方法
    struct MeshQualityCheckResult {
        bool isGoodQuality = false;
        double edgeLengthRatio = 0.0;
        double minEdgeLength = 0.0;
        double maxEdgeLength = 0.0;
        double avgEdgeLength = 0.0;
        std::string errorMessage;
    };
    MeshQualityCheckResult checkMeshQuality();

    // 相机控制
    void updateCamera();
    void resetCamera();

    // 事件处理
    static void mouseCallback(GLFWwindow* window, int button, int action, int mods);
    static void cursorCallback(GLFWwindow* window, double xpos, double ypos);
    static void scrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    static void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods);

    // 工具函数
    glm::vec3 eigenToGLM(const STM::Vector3& v);
    STM::Vector3 glmToEigen(const glm::vec3& v);
    void logMessage(const std::string& message);
    void updateStatistics();

    // 图案生成辅助函数
    void generateGridPattern();
    void generateHexagonalPattern();
    void generateSpiralPattern();
    void generateHilbertPattern();
    void generateHilbertCurve(int x, int y, int size, int dx, int dy, int order,
                             std::vector<std::pair<int, int>>& points);
    float calculateTotalPathLength();

    // 日志系统
    std::vector<std::string> m_logMessages;
    bool m_autoScroll = true;
};

} // namespace SurfaceTextureMapping