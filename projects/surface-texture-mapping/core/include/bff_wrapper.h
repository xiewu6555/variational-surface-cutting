#pragma once

// BFF算法包装器 - 集成boundary-first-flattening库
// 提供geometry-central到BFF的数据转换接口

#include <vector>
#include <memory>
#include <geometrycentral/surface/halfedge_mesh.h>
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <geometrycentral/utilities/vector2.h>

namespace SurfaceTextureMapping {

// 类型别名 - 避免using namespace导致的命名冲突
using HalfedgeMesh = geometrycentral::surface::ManifoldSurfaceMesh;
using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
using Vertex = geometrycentral::surface::Vertex;
using Edge = geometrycentral::surface::Edge;
using Vector2 = geometrycentral::Vector2;

// BFF UV展开结果
struct BFFResult {
    bool success;
    std::vector<Vector2> uvCoordinates;  // 每个顶点的UV坐标
    double distortion;                    // 参数化失真度
    std::string errorMessage;
};

// BFF参数配置
struct BFFConfig {
    enum class FlatteningMode {
        AUTOMATIC,          // 自动最小化面积失真
        WITH_CONES,        // 指定锥点位置
        TO_DISK,           // 展开到单位圆盘
        TO_SPHERE          // 映射到球面（genus 0网格）
    };

    FlatteningMode mode = FlatteningMode::AUTOMATIC;
    std::vector<size_t> coneVertices;     // 锥点的顶点索引
    std::vector<double> coneAngles;       // 锥点的目标角度（弧度）
    bool prescribeBoundaryAngles = false;  // 是否指定边界角度
    std::vector<double> boundaryAngles;    // 边界顶点的目标角度
};

// BFF算法包装器
class BFFWrapper {
public:
    BFFWrapper();
    ~BFFWrapper();

    // 设置输入网格（从geometry-central格式）
    bool setMesh(HalfedgeMesh* mesh, VertexPositionGeometry* geometry);

    // 应用切割路径（将网格沿切割线分开）
    bool applyCuts(const std::vector<std::vector<Edge>>& cutPaths);

    // 执行BFF参数化
    BFFResult computeParameterization(const BFFConfig& config = BFFConfig());

    // 获取失真度量
    double computeAngleDistortion() const;
    double computeAreaDistortion() const;
    double computeConformalError() const;

    // 获取UV边界
    std::vector<std::vector<Vector2>> getUVBoundaries() const;

    // 调试和可视化
    std::vector<std::array<Vector2, 3>> getUVTriangles() const;
    void exportUVMesh(const std::string& filename) const;

private:
    class Implementation;
    std::unique_ptr<Implementation> m_impl;

    // 内部数据
    HalfedgeMesh* m_gcMesh;
    VertexPositionGeometry* m_gcGeometry;
    std::vector<Vector2> m_uvCoordinates;
    bool m_hasCuts;
    bool m_isParameterized;
};

// MeshConverter类已移动到real_algorithm_integration.h中以避免重复定义

// 智能的切割生成器（如果需要自动生成切割）
class SmartCutGenerator {
public:
    // 为闭合网格生成合适的切割线
    static std::vector<std::vector<Edge>> generateCutsForClosedMesh(
        HalfedgeMesh* mesh,
        VertexPositionGeometry* geometry,
        const std::vector<Vertex>& highCurvaturePoints
    );

    // 检测并标记锥点
    static std::vector<Vertex> detectConePoints(
        HalfedgeMesh* mesh,
        VertexPositionGeometry* geometry,
        double curvatureThreshold = 0.5
    );

private:
    // 使用最短路径算法连接锥点
    static std::vector<Edge> findShortestPath(
        HalfedgeMesh* mesh,
        VertexPositionGeometry* geometry,
        Vertex start,
        Vertex end
    );
};

} // namespace SurfaceTextureMapping