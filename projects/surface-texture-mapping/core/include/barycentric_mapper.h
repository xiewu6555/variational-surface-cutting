#pragma once

/**
 * 重心坐标映射器
 * 提供UV空间点与3D曲面点之间的双向映射
 * 基于重心坐标插值实现高效、准确的点对点映射
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <vector>
#include <memory>
#include <optional>
#include <Eigen/Core>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "texture_mapping.h"

namespace SurfaceTextureMapping {

/**
 * 重心坐标映射器类
 *
 * 核心功能：
 * 1. UV点 -> 3D点: 找到UV点所在的UV三角形，计算重心坐标，插值得到3D坐标
 * 2. 3D点 -> UV点: 找到3D点所在的3D三角形，计算重心坐标，插值得到UV坐标
 *
 * 算法要点：
 * - 重心坐标计算: 对于三角形ABC和点P, 计算 (α, β, γ) 使得 P = αA + βB + γC
 * - 点在三角形内判定: α,β,γ ≥ 0 且 α+β+γ = 1
 * - 空间索引: 使用BVH加速三角形查找
 */
class BarycentricMapper {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector2 = geometrycentral::Vector2;
    using Vector3 = geometrycentral::Vector3;
    using Face = geometrycentral::surface::Face;
    using Vertex = geometrycentral::surface::Vertex;
    using UVMapping = TextureMapper::UVMapping;

    /**
     * 重心坐标表示
     */
    struct BarycentricCoord {
        Face face;             // 所在三角形
        double alpha;          // 第一个顶点权重
        double beta;           // 第二个顶点权重
        double gamma;          // 第三个顶点权重
        bool isValid;          // 坐标是否有效

        // 便捷构造
        BarycentricCoord() : isValid(false), alpha(0), beta(0), gamma(0) {}

        // 验证: α+β+γ = 1 且 α,β,γ ≥ 0
        bool validate() const {
            constexpr double eps = 1e-10;
            return isValid &&
                   alpha >= -eps && beta >= -eps && gamma >= -eps &&
                   std::abs(alpha + beta + gamma - 1.0) < eps;
        }
    };

    /**
     * 映射查询结果
     */
    struct MappingResult {
        Vector3 point3D;           // 3D坐标
        Vector2 pointUV;           // UV坐标
        BarycentricCoord barycentric; // 重心坐标
        bool success;              // 映射是否成功
        double distanceToNearest;  // 到最近三角形的距离 (如果点在三角形外)
    };

    BarycentricMapper() = default;
    ~BarycentricMapper() = default;

    /**
     * 设置输入网格和UV映射
     * @param mesh 3D网格
     * @param geometry 几何信息
     * @param uvMapping UV坐标映射
     */
    void setInput(std::shared_ptr<SurfaceMesh> mesh,
                  std::shared_ptr<VertexPositionGeometry> geometry,
                  const UVMapping& uvMapping);

    /**
     * 构建空间索引 (加速查询)
     * 内部使用BVH或Grid加速结构
     */
    void buildSpatialIndex();

    /**
     * UV点 -> 3D点映射
     * @param uvPoint UV空间中的点
     * @return 3D空间中的对应点
     */
    std::optional<MappingResult> mapUVto3D(const Vector2& uvPoint);

    /**
     * 3D点 -> UV点映射
     * @param point3D 3D空间中的点
     * @return UV空间中的对应点
     */
    std::optional<MappingResult> map3DtoUV(const Vector3& point3D);

    /**
     * 批量UV点 -> 3D点映射
     * @param uvPoints UV空间中的点集
     * @return 3D空间中的对应点集
     */
    std::vector<MappingResult> batchMapUVto3D(const std::vector<Vector2>& uvPoints);

    /**
     * 批量3D点 -> UV点映射
     * @param points3D 3D空间中的点集
     * @return UV空间中的对应点集
     */
    std::vector<MappingResult> batchMap3DtoUV(const std::vector<Vector3>& points3D);

    /**
     * 计算重心坐标 (给定三角形和点)
     * @param triangle 三角形的三个顶点 (3D或UV空间)
     * @param point 查询点
     * @return 重心坐标 (α, β, γ)
     */
    static Eigen::Vector3d computeBarycentricCoordinates(
        const Eigen::Vector2d& v0,
        const Eigen::Vector2d& v1,
        const Eigen::Vector2d& v2,
        const Eigen::Vector2d& point);

    static Eigen::Vector3d computeBarycentricCoordinates(
        const Eigen::Vector3d& v0,
        const Eigen::Vector3d& v1,
        const Eigen::Vector3d& v2,
        const Eigen::Vector3d& point);

    /**
     * 检查点是否在三角形内
     * @param bary 重心坐标
     * @param epsilon 数值容差
     * @return 是否在三角形内
     */
    static bool isPointInTriangle(const Eigen::Vector3d& bary, double epsilon = 1e-10);

    /**
     * 使用重心坐标插值3D点
     * @param face 三角形
     * @param bary 重心坐标
     * @return 插值得到的3D点
     */
    Vector3 interpolate3DPoint(Face face, const Eigen::Vector3d& bary);

    /**
     * 使用重心坐标插值UV点
     * @param face 三角形
     * @param bary 重心坐标
     * @return 插值得到的UV点
     */
    Vector2 interpolateUVPoint(Face face, const Eigen::Vector3d& bary);

    /**
     * 查找包含UV点的三角形
     * @param uvPoint UV空间中的点
     * @return 包含该点的三角形 (如果存在)
     */
    std::optional<Face> findUVTriangle(const Vector2& uvPoint);

    /**
     * 查找包含3D点的三角形 (最近投影)
     * @param point3D 3D空间中的点
     * @return 最近的三角形和投影点
     */
    std::optional<Face> findNearestTriangle(const Vector3& point3D);

    /**
     * 边界情况处理
     */
    struct BoundaryHandling {
        bool snapToEdge;        // 是否捕捉到边
        bool snapToVertex;      // 是否捕捉到顶点
        double snapTolerance;   // 捕捉容差
    };

    void setBoundaryHandling(const BoundaryHandling& handling) {
        boundaryHandling_ = handling;
    }

private:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
    UVMapping uvMapping_;
    BoundaryHandling boundaryHandling_;

    // 空间索引数据结构
    struct SpatialIndex {
        bool isBuilt = false;
        // BVH或Grid数据 (实现时添加)
    };
    SpatialIndex spatialIndexUV_;
    SpatialIndex spatialIndex3D_;

    /**
     * 内部辅助函数
     */

    // 计算点到三角形的投影和距离
    double distanceToTriangle(const Vector3& point, Face face, Vector3& projection);
    double distanceToTriangleUV(const Vector2& point, Face face, Vector2& projection);

    // 处理边界情况 (点在边上、顶点上)
    void handleBoundaryCase(MappingResult& result);

    // 获取三角形的顶点坐标
    std::array<Vector3, 3> getTriangleVertices3D(Face face);
    std::array<Vector2, 3> getTriangleVerticesUV(Face face);
};

} // namespace SurfaceTextureMapping