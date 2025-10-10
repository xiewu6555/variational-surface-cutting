#pragma once

/**
 * 边界提取器 (Boundary Extractor)
 * 从EulerianShapeOptimizer提取边界线并转换为网格边路径
 *
 * 核心算法：
 * 1. 从优化器获取BoundarySegment列表
 * 2. 将3D边界线snap到最近的网格边
 * 3. 使用Dijkstra算法找到边界点之间的最短路径
 * 4. 映射core边到geometry-central边
 *
 * 版本: 1.0
 * 日期: 2025-10-09
 */

#include <vector>
#include <map>
#include <unordered_map>
#include <queue>
#include <limits>

// geometry-central类型
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

// Core库类型（前向声明）
class HalfedgeMesh;
template<typename T> class Geometry;
struct Vector3;  // IMPORTANT: Vector3 is declared as struct, not class (see vector3.h:8)
class VertexPtr;
class EdgePtr;
class FacePtr;

namespace SurfaceTextureMapping {

/**
 * 边界提取器类
 */
class BoundaryExtractor {
public:
    // 类型别名
    using GC_Mesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using GC_Geometry = geometrycentral::surface::VertexPositionGeometry;
    using GC_Vertex = geometrycentral::surface::Vertex;
    using GC_Edge = geometrycentral::surface::Edge;
    using GC_Vector3 = geometrycentral::Vector3;

    using Core_Mesh = ::HalfedgeMesh;
    using Core_Geometry = Geometry<Vector3>;

    /**
     * 3D边界线（从优化器提取）
     */
    struct BoundaryLine {
        GC_Vector3 start;
        GC_Vector3 end;
        int segmentType;       // BoundarySegment类型
        double length;         // 边界长度
    };

    /**
     * 边路径提取选项
     */
    struct ExtractionOptions {
        double snapTolerance = 1e-3;        // snap到边的容差
        bool useGeodesicPaths = false;      // 使用测地线路径
        bool allowCrossFace = true;         // 允许穿过面内部
        int maxPathLength = 1000;           // 最大路径长度（防止无限循环）
        bool verbose = false;               // 详细日志
    };

    /**
     * 提取结果
     */
    struct ExtractionResult {
        bool success = false;
        std::string errorMessage;

        // 统计
        int numBoundariesProcessed = 0;
        int numEdgesExtracted = 0;
        int numSnappedVertices = 0;

        // 质量指标
        double averageSnapError = 0.0;      // 平均snap误差
        double maxSnapError = 0.0;          // 最大snap误差
    };

    BoundaryExtractor() = default;
    ~BoundaryExtractor() = default;

    /**
     * 主入口：将边界线转换为边路径
     *
     * @param boundaries 边界线列表
     * @param gcMesh geometry-central网格
     * @param gcGeometry geometry-central几何
     * @param coreMesh core网格
     * @param coreGeometry core几何
     * @param options 提取选项
     * @param outResult 输出结果
     * @return GC边路径列表
     */
    static std::vector<std::vector<GC_Edge>> extractEdgePaths(
        const std::vector<BoundaryLine>& boundaries,
        GC_Mesh* gcMesh,
        GC_Geometry* gcGeometry,
        Core_Mesh* coreMesh,
        Core_Geometry* coreGeometry,
        const ExtractionOptions& options,
        ExtractionResult& outResult
    );

private:
    /**
     * 内部方法
     */

    // 找到距离3D点最近的网格顶点（core格式）
    static VertexPtr findNearestVertex(
        const GC_Vector3& point,
        Core_Mesh* mesh,
        Core_Geometry* geometry,
        double& outDistance
    );

    // 使用Dijkstra算法找到两顶点间最短边路径（core格式）
    static std::vector<EdgePtr> findShortestEdgePath(
        VertexPtr start,
        VertexPtr end,
        Core_Mesh* mesh,
        Core_Geometry* geometry,
        int maxSteps
    );

    // 将core边映射到GC边
    static std::vector<GC_Edge> mapCoreEdgesToGC(
        const std::vector<EdgePtr>& coreEdges,
        Core_Mesh* coreMesh,
        GC_Mesh* gcMesh
    );

    // 验证边路径连通性
    static bool validateEdgePath(
        const std::vector<GC_Edge>& path,
        GC_Mesh* mesh
    );

    // 计算路径长度
    static double computePathLength(
        const std::vector<GC_Edge>& path,
        GC_Geometry* geometry
    );

    // 辅助：计算两点距离
    static double distance(const GC_Vector3& a, const GC_Vector3& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    // 辅助：Vector3转换（GC ↔ Core）
    static GC_Vector3 coreToGC(const Vector3& v);
    static Vector3 gcToCore(const GC_Vector3& v);
};

} // namespace SurfaceTextureMapping
