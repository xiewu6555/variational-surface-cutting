/**
 * 图案回映射器实现
 *
 * 版本: 1.1
 * 日期: 2025-09-30
 * 更新: 集成CGAL Surface_mesh_shortest_path测地线算法
 */

#include "pattern_back_mapper.h"
#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <unordered_map>

// CGAL测地线算法
#include <CGAL/Simple_cartesian.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Surface_mesh_shortest_path.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>

using namespace geometrycentral;
using namespace geometrycentral::surface;

// CGAL类型定义
using CGALKernel = CGAL::Simple_cartesian<double>;
using CGALPoint3 = CGALKernel::Point_3;
using CGALMesh = CGAL::Surface_mesh<CGALPoint3>;
using CGALVertex = CGALMesh::Vertex_index;
using CGALFace = CGALMesh::Face_index;
using CGALShortestPath = CGAL::Surface_mesh_shortest_path_traits<CGALKernel, CGALMesh>;
using CGALShortestPathComputer = CGAL::Surface_mesh_shortest_path<CGALShortestPath>;

namespace SurfaceTextureMapping {

// ============================================================================
// CGAL网格转换辅助函数
// ============================================================================

/**
 * 将geometry-central网格转换为CGAL Surface_mesh
 * 用于测地线计算
 */
static std::pair<CGALMesh, std::unordered_map<size_t, CGALVertex>>
convertToCGALMesh(std::shared_ptr<SurfaceMesh> mesh,
                  std::shared_ptr<VertexPositionGeometry> geometry)
{
    CGALMesh cgalMesh;
    std::unordered_map<size_t, CGALVertex> vertexMap;

    // 添加顶点
    for (Vertex v : mesh->vertices()) {
        Vector3 pos = geometry->vertexPositions[v];
        CGALVertex cgalV = cgalMesh.add_vertex(CGALPoint3(pos.x, pos.y, pos.z));
        vertexMap[v.getIndex()] = cgalV;
    }

    // 添加面
    for (Face f : mesh->faces()) {
        std::vector<CGALVertex> faceVertices;
        for (Vertex v : f.adjacentVertices()) {
            faceVertices.push_back(vertexMap[v.getIndex()]);
        }
        cgalMesh.add_face(faceVertices);
    }

    return {cgalMesh, vertexMap};
}

/**
 * 在CGAL网格中查找最近的顶点
 */
static CGALVertex findNearestVertex(const CGALMesh& mesh, const CGALPoint3& point)
{
    CGALVertex nearest;
    double minDist = std::numeric_limits<double>::max();

    for (CGALVertex v : mesh.vertices()) {
        const CGALPoint3& vPos = mesh.point(v);
        double dx = vPos.x() - point.x();
        double dy = vPos.y() - point.y();
        double dz = vPos.z() - point.z();
        double dist = std::sqrt(dx * dx + dy * dy + dz * dz);

        if (dist < minDist) {
            minDist = dist;
            nearest = v;
        }
    }

    return nearest;
}

// ============================================================================
// 公共接口
// ============================================================================

void PatternBackMapper::setInput(
    std::shared_ptr<SurfaceMesh> mesh,
    std::shared_ptr<VertexPositionGeometry> geometry,
    const UVMapping& uvMapping,
    std::shared_ptr<BarycentricMapper> barycentricMapper)
{
    mesh_ = mesh;
    geometry_ = geometry;
    uvMapping_ = uvMapping;
    barycentricMapper_ = barycentricMapper;

    seamMappingBuilt_ = false;
}

void PatternBackMapper::buildSeamMapping()
{
    if (!mesh_ || !geometry_) {
        return;
    }

    seamMappings_.clear();

    // 简化版本: 检测UV坐标不连续的边作为缝边界
    for (Edge edge : mesh_->edges()) {
        if (isSeamEdge(edge)) {
            SeamMapping seam;
            seam.seamEdge = edge;

            // 获取缝边的两个顶点
            auto he = edge.halfedge();
            Vertex v0 = he.tailVertex();
            Vertex v1 = he.tipVertex();

            seam.vertices.push_back(v0);
            seam.vertices.push_back(v1);

            seamMappings_.push_back(seam);
        }
    }

    seamMappingBuilt_ = true;
    std::cout << "检测到 " << seamMappings_.size() << " 条缝边界" << std::endl;
}

std::vector<Vector3> PatternBackMapper::mapPathTo3D(
    const std::vector<Vector2>& uvPath,
    const MappingParams& params)
{
    if (uvPath.empty() || !barycentricMapper_) {
        return {};
    }

    // 特殊处理：单点路径
    if (uvPath.size() == 1) {
        auto result = barycentricMapper_->mapUVto3D(uvPath[0]);
        if (result.has_value()) {
            return {result->point3D};
        }
        return {};
    }

    // 分段UV路径
    auto segments = segmentUVPath(uvPath);

    // 映射每个段到3D
    std::vector<Vector3> path3D;

    for (const auto& segment : segments) {
        auto segment3D = mapSegmentTo3D(segment, params);

        // 合并路径点
        if (path3D.empty()) {
            path3D.insert(path3D.end(), segment3D.points.begin(), segment3D.points.end());
        } else {
            // 避免重复添加连接点
            path3D.insert(path3D.end(), segment3D.points.begin() + 1, segment3D.points.end());
        }
    }

    return path3D;
}

std::vector<std::vector<Vector3>> PatternBackMapper::mapPathsTo3D(
    const std::vector<std::vector<Vector2>>& uvPaths,
    const MappingParams& params)
{
    std::vector<std::vector<Vector3>> paths3D;
    paths3D.reserve(uvPaths.size());

    for (const auto& uvPath : uvPaths) {
        paths3D.push_back(mapPathTo3D(uvPath, params));
    }

    return paths3D;
}

std::vector<PatternBackMapper::UVPathSegment>
PatternBackMapper::segmentUVPath(const std::vector<Vector2>& uvPath)
{
    std::vector<UVPathSegment> segments;

    if (uvPath.size() < 2) {
        return segments;
    }

    for (size_t i = 0; i < uvPath.size() - 1; ++i) {
        UVPathSegment segment;
        segment.start = uvPath[i];
        segment.end = uvPath[i + 1];

        // 查找起点和终点所在的三角形
        auto startFaceOpt = barycentricMapper_->findUVTriangle(segment.start);
        auto endFaceOpt = barycentricMapper_->findUVTriangle(segment.end);

        if (!startFaceOpt.has_value() || !endFaceOpt.has_value()) {
            segment.type = SegmentType::Invalid;
            segments.push_back(segment);
            continue;
        }

        segment.startFace = startFaceOpt.value();
        segment.endFace = endFaceOpt.value();

        // 判断段类型
        if (segment.startFace == segment.endFace) {
            segment.type = SegmentType::IntraTriangle;
        } else {
            // 简化: 所有跨三角形的段都视为普通跨边
            // TODO: 实现缝边界检测
            segment.type = SegmentType::CrossEdge;
        }

        segments.push_back(segment);
    }

    return segments;
}

PatternBackMapper::Path3DSegment PatternBackMapper::mapSegmentTo3D(
    const UVPathSegment& segment,
    const MappingParams& params)
{
    Path3DSegment result;
    result.type = segment.type;
    result.isGeodesic = false;

    if (segment.type == SegmentType::Invalid) {
        return result;
    }

    // 简化版本: 直接映射起点和终点，不处理跨缝情况
    auto startResult = barycentricMapper_->mapUVto3D(segment.start);
    auto endResult = barycentricMapper_->mapUVto3D(segment.end);

    if (!startResult.has_value() || !endResult.has_value()) {
        return result;
    }

    Vector3 start3D = startResult->point3D;
    Vector3 end3D = endResult->point3D;

    if (segment.type == SegmentType::IntraTriangle) {
        // 三角形内路径: 直线插值
        result.points.push_back(start3D);
        result.points.push_back(end3D);
        result.length = distance3D(start3D, end3D);
    } else if (segment.type == SegmentType::CrossSeam && params.useGeodesicPath) {
        // 跨缝路径: 使用测地线
        result.points = computeGeodesicPath(start3D, end3D, params.geodesicResolution);
        result.length = computePathLength(result.points);
        result.isGeodesic = true;
    } else {
        // 普通跨边路径: 直线连接
        result.points.push_back(start3D);
        result.points.push_back(end3D);
        result.length = distance3D(start3D, end3D);
    }

    return result;
}

bool PatternBackMapper::detectSeamCrossing(const UVPathSegment& segment)
{
    // 简化版本: 检查是否被标记为CrossSeam类型
    return segment.type == SegmentType::CrossSeam;
}

std::optional<Vector2> PatternBackMapper::computeSeamIntersection(
    const Vector2& start,
    const Vector2& end,
    const Edge& seamEdge)
{
    // 获取缝边的UV坐标
    auto he = seamEdge.halfedge();
    Vertex v0 = he.tailVertex();
    Vertex v1 = he.tipVertex();

    size_t idx0 = v0.getIndex();
    size_t idx1 = v1.getIndex();

    if (idx0 >= uvMapping_.uvCoordinates.size() ||
        idx1 >= uvMapping_.uvCoordinates.size()) {
        return std::nullopt;
    }

    Vector2 seamStart = uvMapping_.uvCoordinates[idx0];
    Vector2 seamEnd = uvMapping_.uvCoordinates[idx1];

    // 计算线段交点
    return lineSegmentIntersection(start, end, seamStart, seamEnd);
}

std::optional<Vector2> PatternBackMapper::findCorrespondingPointAcrossSeam(
    const Vector2& uvPoint,
    const Edge& seamEdge)
{
    // 简化版本: 返回相同点
    // TODO: 实现真正的缝对应点查找
    return uvPoint;
}

std::vector<Vector3> PatternBackMapper::computeGeodesicPath(
    const Vector3& start,
    const Vector3& end,
    double resolution)
{
    // 使用CGAL Surface_mesh_shortest_path计算真实测地线
    return computeGeodesicPathCGAL(start, end, resolution);
}

PatternBackMapper::MappingQuality PatternBackMapper::evaluateMappingQuality(
    const std::vector<Vector2>& uvPath,
    const std::vector<Vector3>& path3D)
{
    MappingQuality quality;

    // 计算UV路径长度
    quality.totalLengthUV = 0.0;
    for (size_t i = 0; i < uvPath.size() - 1; ++i) {
        quality.totalLengthUV += distance2D(uvPath[i], uvPath[i + 1]);
    }

    // 计算3D路径长度
    quality.totalLength3D = computePathLength(path3D);

    // 计算长度偏差
    if (quality.totalLengthUV > 0) {
        quality.lengthDeviation = std::abs(quality.totalLength3D - quality.totalLengthUV) /
                                 quality.totalLengthUV * 100.0;
    } else {
        quality.lengthDeviation = 0.0;
    }

    quality.isLengthConsistent = quality.lengthDeviation < 1.0;  // 1% 容差

    // 统计跨缝和测地线段数量
    auto segments = segmentUVPath(uvPath);
    quality.numSeamCrossings = 0;
    quality.numGeodesicSegments = 0;

    for (const auto& segment : segments) {
        if (segment.type == SegmentType::CrossSeam) {
            quality.numSeamCrossings++;
            quality.numGeodesicSegments++;
        }
    }

    return quality;
}

// ============================================================================
// 私有辅助函数
// ============================================================================

bool PatternBackMapper::isSeamEdge(const Edge& edge)
{
    // 简化版本: 检查边两侧的面是否有UV坐标不连续
    // 真实实现需要比较UV坐标的差异

    auto he = edge.halfedge();
    if (!he.isInterior()) {
        return false;  // 边界边不是缝
    }

    // 获取边的两个顶点
    Vertex v0 = he.tailVertex();
    Vertex v1 = he.tipVertex();

    size_t idx0 = v0.getIndex();
    size_t idx1 = v1.getIndex();

    if (idx0 >= uvMapping_.uvCoordinates.size() ||
        idx1 >= uvMapping_.uvCoordinates.size()) {
        return false;
    }

    // 简化判定: 假设没有缝 (BFF展平后通常是单张UV图)
    // TODO: 实现真正的UV不连续性检测
    return false;
}

double PatternBackMapper::distance2D(const Vector2& a, const Vector2& b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    return std::sqrt(dx * dx + dy * dy);
}

double PatternBackMapper::distance3D(const Vector3& a, const Vector3& b)
{
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double dz = b.z - a.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::optional<Vector2> PatternBackMapper::lineSegmentIntersection(
    const Vector2& a1, const Vector2& a2,
    const Vector2& b1, const Vector2& b2)
{
    // 计算线段 (a1, a2) 和 (b1, b2) 的交点

    double x1 = a1.x, y1 = a1.y;
    double x2 = a2.x, y2 = a2.y;
    double x3 = b1.x, y3 = b1.y;
    double x4 = b2.x, y4 = b2.y;

    double denom = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);

    if (std::abs(denom) < 1e-10) {
        // 平行或重合
        return std::nullopt;
    }

    double t = ((x1 - x3) * (y3 - y4) - (y1 - y3) * (x3 - x4)) / denom;
    double u = -((x1 - x2) * (y1 - y3) - (y1 - y2) * (x1 - x3)) / denom;

    // 检查交点是否在两条线段内
    if (t >= 0.0 && t <= 1.0 && u >= 0.0 && u <= 1.0) {
        Vector2 intersection;
        intersection.x = x1 + t * (x2 - x1);
        intersection.y = y1 + t * (y2 - y1);
        return intersection;
    }

    return std::nullopt;
}

std::vector<Vector3> PatternBackMapper::resamplePath(
    const std::vector<Vector3>& path,
    double resolution)
{
    if (path.size() < 2) {
        return path;
    }

    std::vector<Vector3> resampled;
    resampled.push_back(path[0]);

    double accumulated = 0.0;

    for (size_t i = 0; i < path.size() - 1; ++i) {
        double segmentLength = distance3D(path[i], path[i + 1]);
        accumulated += segmentLength;

        // 在当前段上采样点
        int numSamples = static_cast<int>(segmentLength / resolution);
        for (int j = 1; j <= numSamples; ++j) {
            double t = static_cast<double>(j) / (numSamples + 1);
            Vector3 point;
            point.x = path[i].x + t * (path[i + 1].x - path[i].x);
            point.y = path[i].y + t * (path[i + 1].y - path[i].y);
            point.z = path[i].z + t * (path[i + 1].z - path[i].z);
            resampled.push_back(point);
        }

        resampled.push_back(path[i + 1]);
    }

    return resampled;
}

double PatternBackMapper::computePathLength(const std::vector<Vector3>& path)
{
    if (path.size() < 2) {
        return 0.0;
    }

    double length = 0.0;
    for (size_t i = 0; i < path.size() - 1; ++i) {
        length += distance3D(path[i], path[i + 1]);
    }

    return length;
}

std::vector<Vector3> PatternBackMapper::computeGeodesicPathCGAL(
    const Vector3& start,
    const Vector3& end,
    double resolution)
{
    if (!mesh_ || !geometry_) {
        std::cerr << "PatternBackMapper: 网格未初始化，无法计算测地线" << std::endl;
        return {};
    }

    try {
        // 1. 转换geometry-central网格为CGAL格式
        auto [cgalMesh, vertexMap] = convertToCGALMesh(mesh_, geometry_);

        if (cgalMesh.number_of_vertices() == 0) {
            std::cerr << "PatternBackMapper: CGAL网格转换失败" << std::endl;
            return {};
        }

        // 2. 创建CGAL测地线计算器
        CGALShortestPathComputer shortestPath(cgalMesh);

        // 3. 查找起点和终点最近的顶点
        CGALPoint3 cgalStart(start.x, start.y, start.z);
        CGALPoint3 cgalEnd(end.x, end.y, end.z);

        CGALVertex startVertex = findNearestVertex(cgalMesh, cgalStart);
        CGALVertex endVertex = findNearestVertex(cgalMesh, cgalEnd);

        // 4. 设置起点并计算到终点的最短路径
        shortestPath.add_source_point(startVertex);

        std::vector<CGALPoint3> cgalPath;
        shortestPath.shortest_path_points_to_source_points(endVertex, std::back_inserter(cgalPath));

        // 5. 如果路径为空，尝试直接连接（可能在同一顶点）
        if (cgalPath.empty()) {
            std::vector<Vector3> directPath;
            directPath.push_back(start);
            directPath.push_back(end);
            return directPath;
        }

        // 6. 转换CGAL路径回geometry-central格式
        std::vector<Vector3> path;
        for (const auto& cgalPoint : cgalPath) {
            Vector3 point;
            point.x = cgalPoint.x();
            point.y = cgalPoint.y();
            point.z = cgalPoint.z();
            path.push_back(point);
        }

        // 7. 重采样路径以满足分辨率要求
        if (resolution > 0 && path.size() > 1) {
            path = resamplePath(path, resolution);
        }

        // 8. 确保起点和终点精确匹配输入点
        if (!path.empty()) {
            path.front() = start;
            path.back() = end;
        }

        return path;

    } catch (const std::exception& e) {
        std::cerr << "CGAL测地线计算异常: " << e.what() << std::endl;

        // 降级为直线连接
        std::vector<Vector3> fallbackPath;
        fallbackPath.push_back(start);
        fallbackPath.push_back(end);
        return fallbackPath;
    }
}

std::vector<Vector3> PatternBackMapper::computeGeodesicPathGC(
    const Vector3& start,
    const Vector3& end,
    double resolution)
{
    // geometry-central没有内置测地线算法
    // 使用CGAL实现作为统一后端
    return computeGeodesicPathCGAL(start, end, resolution);
}

} // namespace SurfaceTextureMapping