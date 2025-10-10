#pragma once

/**
 * 图案回映射器 (Pattern Back Mapper)
 * 将UV空间的图案路径映射回3D曲面，处理跨缝边界和测地线路径
 * 这是Real-Space UV展开的核心组件，确保图案在3D表面连续
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <vector>
#include <memory>
#include <optional>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "texture_mapping.h"
#include "barycentric_mapper.h"

namespace SurfaceTextureMapping {

/**
 * 图案回映射器类
 *
 * 核心挑战:
 * 1. UV空间的路径可能跨越切缝边界
 * 2. 需要在3D表面重建连续路径 (使用测地线)
 * 3. 保持Real-Space尺度 (1 UV单位 = 1 mm)
 *
 * 算法流程:
 * 1. 将UV路径分段为三角形内路径
 * 2. 检测路径与缝边界的交叉点
 * 3. 在缝两侧找到对应点
 * 4. 使用测地线算法连接跨缝路径
 * 5. 合并为完整的3D路径
 */
class PatternBackMapper {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector2 = geometrycentral::Vector2;
    using Vector3 = geometrycentral::Vector3;
    using Face = geometrycentral::surface::Face;
    using Edge = geometrycentral::surface::Edge;
    using Vertex = geometrycentral::surface::Vertex;
    using UVMapping = TextureMapper::UVMapping;

    /**
     * 路径段类型
     */
    enum class SegmentType {
        IntraTriangle,    // 三角形内路径 (直接插值)
        CrossEdge,        // 跨边路径 (普通边)
        CrossSeam,        // 跨缝路径 (需要测地线连接)
        Invalid           // 无效路径
    };

    /**
     * UV路径段
     */
    struct UVPathSegment {
        Vector2 start;
        Vector2 end;
        Face startFace;
        Face endFace;
        SegmentType type;

        // 如果是跨缝段，记录缝边界信息
        std::optional<Edge> seamEdge;
        Vector2 seamCrossingPoint;  // 与缝的交点 (UV空间)
    };

    /**
     * 3D路径段
     */
    struct Path3DSegment {
        std::vector<Vector3> points;  // 3D点序列
        SegmentType type;
        double length;                // 实际3D长度 (mm)
        bool isGeodesic;              // 是否为测地线路径
    };

    /**
     * 缝映射信息
     */
    struct SeamMapping {
        Edge seamEdge;                 // 缝边
        std::vector<Vertex> vertices;  // 缝两侧的顶点对

        // UV空间中，同一个3D点可能有两个UV坐标 (缝两侧)
        struct DuplicateUVPair {
            Vector2 uvLeft;
            Vector2 uvRight;
            Vertex vertex3D;
        };
        std::vector<DuplicateUVPair> uvPairs;
    };

    /**
     * 映射参数
     */
    struct MappingParams {
        bool useGeodesicPath = true;        // 跨缝时使用测地线
        double geodesicResolution = 0.1;    // 测地线采样分辨率 (mm)
        bool preserveRealSpace = true;      // 保持Real-Space尺度
        double seamTolerance = 1e-6;        // 缝边界检测容差
        int maxGeodesicIterations = 100;    // 测地线算法最大迭代
    };

    PatternBackMapper() = default;
    ~PatternBackMapper() = default;

    /**
     * 设置输入数据
     * @param mesh 3D网格
     * @param geometry 几何信息
     * @param uvMapping UV映射
     * @param barycentricMapper 重心坐标映射器 (用于UV->3D转换)
     */
    void setInput(std::shared_ptr<SurfaceMesh> mesh,
                  std::shared_ptr<VertexPositionGeometry> geometry,
                  const UVMapping& uvMapping,
                  std::shared_ptr<BarycentricMapper> barycentricMapper);

    /**
     * 构建缝映射表
     * 分析UV映射，识别所有切缝边界及其对应关系
     */
    void buildSeamMapping();

    /**
     * 将单条UV路径映射回3D
     * @param uvPath UV空间中的路径 (点序列)
     * @param params 映射参数
     * @return 3D空间中的路径
     */
    std::vector<Vector3> mapPathTo3D(
        const std::vector<Vector2>& uvPath,
        const MappingParams& params = MappingParams{});

    /**
     * 批量映射多条路径
     * @param uvPaths 多条UV路径
     * @param params 映射参数
     * @return 多条3D路径
     */
    std::vector<std::vector<Vector3>> mapPathsTo3D(
        const std::vector<std::vector<Vector2>>& uvPaths,
        const MappingParams& params = MappingParams{});

    /**
     * 检测UV路径是否跨越缝边界
     * @param segment UV路径段
     * @return 是否跨缝及交点信息
     */
    bool detectSeamCrossing(const UVPathSegment& segment);

    /**
     * 计算路径与缝边界的交点
     * @param start 路径起点 (UV)
     * @param end 路径终点 (UV)
     * @param seamEdge 缝边
     * @return 交点 (UV坐标)
     */
    std::optional<Vector2> computeSeamIntersection(
        const Vector2& start,
        const Vector2& end,
        const Edge& seamEdge);

    /**
     * 在缝两侧找到对应点
     * @param uvPoint 缝上的UV点 (一侧)
     * @param seamEdge 缝边
     * @return 另一侧的对应UV点
     */
    std::optional<Vector2> findCorrespondingPointAcrossSeam(
        const Vector2& uvPoint,
        const Edge& seamEdge);

    /**
     * 计算两点间的测地线路径
     * 使用CGAL或geometry-central的测地线算法
     * @param start 起点 (3D)
     * @param end 终点 (3D)
     * @param resolution 采样分辨率 (mm)
     * @return 测地线路径点序列
     */
    std::vector<Vector3> computeGeodesicPath(
        const Vector3& start,
        const Vector3& end,
        double resolution = 0.1);

    /**
     * 将UV路径段化为三角形内段
     * @param uvPath 完整UV路径
     * @return 路径段列表
     */
    std::vector<UVPathSegment> segmentUVPath(const std::vector<Vector2>& uvPath);

    /**
     * 映射单个路径段到3D
     * @param segment UV路径段
     * @param params 映射参数
     * @return 3D路径段
     */
    Path3DSegment mapSegmentTo3D(
        const UVPathSegment& segment,
        const MappingParams& params);

    /**
     * 获取所有缝边界
     * @return 缝映射信息列表
     */
    const std::vector<SeamMapping>& getSeamMappings() const {
        return seamMappings_;
    }

    /**
     * 验证映射质量
     */
    struct MappingQuality {
        double totalLength3D;           // 3D路径总长度 (mm)
        double totalLengthUV;           // UV路径总长度 (mm，应该相等)
        double lengthDeviation;         // 长度偏差 (%)
        int numSeamCrossings;           // 跨缝次数
        int numGeodesicSegments;        // 测地线段数量
        bool isLengthConsistent;        // 长度是否一致 (偏差 < 1%)
    };

    MappingQuality evaluateMappingQuality(
        const std::vector<Vector2>& uvPath,
        const std::vector<Vector3>& path3D);

private:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
    UVMapping uvMapping_;
    std::shared_ptr<BarycentricMapper> barycentricMapper_;

    std::vector<SeamMapping> seamMappings_;
    bool seamMappingBuilt_ = false;

    /**
     * 内部辅助函数
     */

    // 检测边是否为缝边界
    bool isSeamEdge(const Edge& edge);

    // 计算两点间的直线距离 (UV或3D)
    double distance2D(const Vector2& a, const Vector2& b);
    double distance3D(const Vector3& a, const Vector3& b);

    // 计算线段与线段的交点 (2D)
    std::optional<Vector2> lineSegmentIntersection(
        const Vector2& a1, const Vector2& a2,
        const Vector2& b1, const Vector2& b2);

    // 路径重采样 (保持固定间距)
    std::vector<Vector3> resamplePath(
        const std::vector<Vector3>& path,
        double resolution);

    // 计算路径总长度
    double computePathLength(const std::vector<Vector3>& path);

    // CGAL测地线算法集成
    std::vector<Vector3> computeGeodesicPathCGAL(
        const Vector3& start,
        const Vector3& end,
        double resolution);

    // geometry-central测地线算法集成
    std::vector<Vector3> computeGeodesicPathGC(
        const Vector3& start,
        const Vector3& end,
        double resolution);
};

} // namespace SurfaceTextureMapping