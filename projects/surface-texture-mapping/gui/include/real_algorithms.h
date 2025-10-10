#pragma once

// 真实算法实现的接口定义
// 逐步集成variational cutting和boundary-first-flattening算法

#include <vector>
#include <memory>
#include <map>
#include <geometrycentral/surface/halfedge_mesh.h>
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <geometrycentral/surface/edge_length_geometry.h>
#include <geometrycentral/surface/intrinsic_geometry_interface.h>
#include <geometrycentral/surface/simple_polygon_mesh.h>
#include <geometrycentral/surface/meshio.h>
#include <geometrycentral/surface/heat_method_distance.h>
#include <geometrycentral/surface/direction_fields.h>
#include <geometrycentral/numerical/linear_algebra_utilities.h>
#include <geometrycentral/numerical/linear_solvers.h>

namespace SurfaceTextureMapping {

using namespace geometrycentral;
using namespace geometrycentral::surface;

// 切割路径表示
struct CutPath {
    std::vector<Edge> edges;           // 组成切割路径的边
    std::vector<Vector3> positions;    // 切割路径上的3D位置（用于渲染）
    double totalLength;                 // 路径总长度
    bool isClosed;                      // 是否闭合路径
};

// 网格质量分析结果
struct MeshAnalysis {
    // 基本拓扑信息
    size_t nVertices;
    size_t nFaces;
    size_t nEdges;
    int eulerCharacteristic;
    int genus;

    // 几何度量
    double surfaceArea;
    double avgEdgeLength;
    double minEdgeLength;
    double maxEdgeLength;

    // 质量指标
    double avgTriangleQuality;  // 基于边长比的三角形质量
    size_t nDegenerateTriangles;
    bool isManifold;
    bool isWatertight;

    // 曲率信息
    double avgMeanCurvature;
    double avgGaussianCurvature;
    std::vector<double> vertexCurvatures;  // 每个顶点的高斯曲率
};

// 变分切割算法实现
class VariationalCuttingAlgorithm {
public:
    VariationalCuttingAlgorithm(HalfedgeMesh* mesh, VertexPositionGeometry* geometry);

    // 步骤1：分析网格质量
    MeshAnalysis analyzeMesh();

    // 步骤2：计算曲率并识别特征点
    std::vector<Vertex> computeFeaturePoints(double curvatureThreshold = 0.8);

    // 步骤3：生成初始切割路径
    std::vector<CutPath> generateInitialCuts(const std::vector<Vertex>& featurePoints);

    // 步骤4：优化切割路径（变分方法）
    void optimizeCuts(std::vector<CutPath>& cuts, int iterations = 10);

    // 步骤5：应用切割到网格
    void applyCutsToMesh(const std::vector<CutPath>& cuts);

    // 获取当前状态的可视化数据
    std::vector<std::vector<Vector3>> getCutLinesForRendering() const;
    std::vector<Vector3> getFeaturePointsForRendering() const;
    std::vector<double> getCurvatureColorsForRendering() const;

private:
    HalfedgeMesh* m_mesh;
    VertexPositionGeometry* m_geometry;
    std::unique_ptr<IntrinsicGeometryInterface> m_intrinsicGeometry;

    // 缓存的计算结果
    VertexData<double> m_gaussianCurvatures;
    VertexData<double> m_meanCurvatures;
    VertexData<double> m_principalCurvatures;
    std::vector<Vertex> m_featurePoints;
    std::vector<CutPath> m_currentCuts;

    // geometry-central工具
    std::unique_ptr<HeatMethodDistanceSolver> m_heatSolver;

    // 内部辅助方法
    double computePathEnergy(const CutPath& path);
    void smoothPath(CutPath& path);
    CutPath findShortestPath(Vertex start, Vertex end);
    CutPath findGeodesicPath(Vertex start, Vertex end);
    double computeVertexFeatureStrength(Vertex v);
    void initializeGeometryTools();
    double computeMeanEdgeLength() const;
    Halfedge findConnectingHalfedge(Vertex v1, Vertex v2) const;

    // 变分优化方法
    std::vector<Vector3> computePathEnergyGradients(const CutPath& path);
    Vector3 projectToMeshSurface(const Vector3& position);
    void updateCutPathGeometry(CutPath& path);
    void removeSelfIntersections(CutPath& path);
    double pointToLineSegmentDistance(const Vector3& point, const Vector3& lineStart, const Vector3& lineEnd);
};

// UV展开算法实现（集成真实BFF算法）
class UVUnwrappingAlgorithm {
public:
    UVUnwrappingAlgorithm(HalfedgeMesh* mesh, VertexPositionGeometry* geometry);

    // 执行UV展开
    bool computeUVCoordinates(const std::vector<CutPath>& cuts);

    // 获取UV坐标
    std::vector<Vector2> getUVCoordinates() const;

    // 计算失真度量
    double computeDistortion() const;
    double computeAngleDistortion() const;
    double computeAreaDistortion() const;

    // 可视化辅助
    std::vector<std::array<Vector2, 3>> getUVTrianglesForRendering() const;

    // 设置是否使用真实BFF算法
    void setUseBFF(bool useBFF) { m_useBFF = useBFF; }

private:
    HalfedgeMesh* m_mesh;
    VertexPositionGeometry* m_geometry;
    VertexData<Vector2> m_uvCoordinates;

    // 边界信息
    std::vector<std::vector<Vertex>> m_boundaryLoops;
    bool m_hasCuts;
    bool m_useBFF; // 是否使用真实BFF算法

    // 内部计算方法
    void computeConformalParameterization();
    void minimizeAreaDistortion();

    // 改进的参数化方法
    bool computeLSCMParameterization(const std::vector<CutPath>& cuts);
    bool computeHarmonicParameterization();
    bool computeRealBFFParameterization(const std::vector<CutPath>& cuts); // 新增：真实BFF
    void fixBoundaryCoordinates();
    void applyCutConstraints(const std::vector<CutPath>& cuts);

    // 失真分析
    double computeFaceAngleDistortion(Face f) const;
    double computeFaceAreaDistortion(Face f) const;

    // 辅助方法
    void detectBoundaryLoops();
    bool isValidParameterization() const;
};

// 算法流水线管理器
class AlgorithmPipeline {
public:
    AlgorithmPipeline();

    // 设置输入网格
    bool loadMesh(const std::string& filename);
    bool setMesh(HalfedgeMesh* mesh, VertexPositionGeometry* geometry);

    // 执行算法步骤（每步都可以独立调用和可视化）
    MeshAnalysis runMeshAnalysis();
    std::vector<Vertex> runFeatureDetection(double threshold = 0.8);
    std::vector<CutPath> runInitialCutGeneration();
    void runCutOptimization(int iterations = 10);
    bool runUVUnwrapping();

    // 获取可视化数据
    struct VisualizationData {
        // 原始网格
        std::vector<Vector3> vertices;
        std::vector<std::array<int, 3>> faces;

        // 特征和曲率
        std::vector<Vector3> featurePoints;
        std::vector<double> curvatureColors;

        // 切割线
        std::vector<std::vector<Vector3>> cutLines;

        // UV展开结果
        std::vector<Vector2> uvCoordinates;
        std::vector<std::array<Vector2, 3>> uvTriangles;

        // 统计信息
        MeshAnalysis analysis;
        double uvDistortion;
    };

    VisualizationData getVisualizationData() const;

    // 获取当前状态
    enum class PipelineState {
        EMPTY,
        MESH_LOADED,
        ANALYZED,
        FEATURES_DETECTED,
        CUTS_GENERATED,
        CUTS_OPTIMIZED,
        UV_UNWRAPPED
    };

    PipelineState getCurrentState() const { return m_state; }

    // 处理状态
    struct ProcessingStatus {
        bool meshLoaded = false;
        bool analysisComplete = false;
        bool cutsGenerated = false;
        bool uvMappingComplete = false;
        size_t numVertices = 0;
        size_t numFaces = 0;
        size_t numCuts = 0;
    };

    ProcessingStatus getStatus() const;

private:
    // 网格数据 (拥有所有权)
    std::unique_ptr<HalfedgeMesh> m_mesh;
    std::unique_ptr<VertexPositionGeometry> m_geometry;

    // 网格数据 (外部指针)
    HalfedgeMesh* m_meshPtr = nullptr;
    VertexPositionGeometry* m_geometryPtr = nullptr;

    // 算法实例
    std::unique_ptr<VariationalCuttingAlgorithm> m_cuttingAlgo;
    std::unique_ptr<UVUnwrappingAlgorithm> m_uvAlgo;

    // 状态和数据
    PipelineState m_state = PipelineState::EMPTY;
    VisualizationData m_visData;
    std::vector<CutPath> m_currentCuts;

    void updateVisualizationData();
};

} // namespace SurfaceTextureMapping