#pragma once

/**
 * 网格预处理器
 * 提供网格清理、重网格化、简化等预处理功能
 */

#include <memory>
#include <vector>
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

namespace SurfaceTextureMapping {

class MeshProcessor {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector3 = geometrycentral::Vector3;
    using Vertex = geometrycentral::surface::Vertex;
    using Edge = geometrycentral::surface::Edge;
    using Face = geometrycentral::surface::Face;

    struct ProcessingParams {
        // 重网格化参数
        double targetEdgeLength = -1.0;     // 目标边长（-1表示自动计算）
        int iterations = 5;                 // 迭代次数
        double smoothingWeight = 0.1;       // 平滑权重
        double featureAngle = 45.0;         // 特征角度（度）

        // 简化参数
        double targetReduction = 0.5;       // 目标减少率
        bool preserveBoundary = true;       // 保留边界
        bool preserveTopology = true;       // 保留拓扑

        // 清理参数
        bool removeIsolatedVertices = true; // 移除孤立顶点
        bool removeDegenerateFaces = true;  // 移除退化面
        bool makeManifold = true;          // 确保流形性质
    };

    struct ProcessingStats {
        int originalVertices;
        int originalFaces;
        int processedVertices;
        int processedFaces;
        double meanEdgeLength;
        double minEdgeLength;
        double maxEdgeLength;
        bool isManifold;
        bool isClosed;
        int boundaryLoops;
    };

    MeshProcessor() = default;
    ~MeshProcessor() = default;

    /**
     * 设置输入网格
     */
    void setMesh(std::shared_ptr<SurfaceMesh> mesh,
                 std::shared_ptr<VertexPositionGeometry> geometry);

    /**
     * 等距重网格化
     * 使边长尽可能均匀
     */
    std::pair<std::shared_ptr<SurfaceMesh>, std::shared_ptr<VertexPositionGeometry>>
    remeshIsotropic(const ProcessingParams& params = ProcessingParams{});

    /**
     * 网格简化
     * 减少面数同时保持形状
     */
    std::pair<std::shared_ptr<SurfaceMesh>, std::shared_ptr<VertexPositionGeometry>>
    simplify(const ProcessingParams& params = ProcessingParams{});

    /**
     * 网格清理
     * 修复常见的网格问题
     */
    std::pair<std::shared_ptr<SurfaceMesh>, std::shared_ptr<VertexPositionGeometry>>
    cleanup(const ProcessingParams& params = ProcessingParams{});

    /**
     * 拉普拉斯平滑
     */
    void smoothLaplacian(int iterations = 5, double weight = 0.5);

    /**
     * 特征保持平滑
     */
    void smoothBilateral(int iterations = 5, double spatialSigma = 1.0, double normalSigma = 0.5);

    /**
     * 计算处理统计
     */
    ProcessingStats computeStats() const;

    /**
     * 检测尖锐特征边
     */
    std::vector<Edge> detectFeatureEdges(double angleThreshold = 30.0) const;

    /**
     * 检测边界环
     */
    std::vector<std::vector<Vertex>> detectBoundaryLoops() const;

private:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;

    // 内部辅助函数
    double computeMeanEdgeLength() const;
    void collapseShortEdges(double minLength);
    void splitLongEdges(double maxLength);
    void flipEdges();
    void tangentialSmoothing(double weight);
    bool isFeatureEdge(const Edge& e, double angleThreshold) const;
};

} // namespace SurfaceTextureMapping