/**
 * 重心坐标映射器实现
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include "barycentric_mapper.h"
#include <algorithm>
#include <limits>
#include <cmath>
#include <iostream>

using namespace geometrycentral;
using namespace geometrycentral::surface;

namespace SurfaceTextureMapping {

// ============================================================================
// 公共接口
// ============================================================================

void BarycentricMapper::setInput(
    std::shared_ptr<SurfaceMesh> mesh,
    std::shared_ptr<VertexPositionGeometry> geometry,
    const UVMapping& uvMapping)
{
    mesh_ = mesh;
    geometry_ = geometry;
    uvMapping_ = uvMapping;

    // 重置空间索引
    spatialIndexUV_.isBuilt = false;
    spatialIndex3D_.isBuilt = false;
}

void BarycentricMapper::buildSpatialIndex()
{
    // 简化版本: 暂时不使用BVH，直接线性搜索
    // TODO: 实现BVH加速结构
    spatialIndexUV_.isBuilt = true;
    spatialIndex3D_.isBuilt = true;
}

std::optional<BarycentricMapper::MappingResult>
BarycentricMapper::mapUVto3D(const Vector2& uvPoint)
{
    if (!mesh_ || !geometry_) {
        return std::nullopt;
    }

    // 查找包含UV点的三角形
    auto faceOpt = findUVTriangle(uvPoint);
    if (!faceOpt.has_value()) {
        return std::nullopt;
    }

    Face face = faceOpt.value();

    // 获取三角形UV顶点
    auto uvVertices = getTriangleVerticesUV(face);

    // 计算重心坐标
    Eigen::Vector2d uv0(uvVertices[0].x, uvVertices[0].y);
    Eigen::Vector2d uv1(uvVertices[1].x, uvVertices[1].y);
    Eigen::Vector2d uv2(uvVertices[2].x, uvVertices[2].y);
    Eigen::Vector2d uvp(uvPoint.x, uvPoint.y);

    Eigen::Vector3d bary = computeBarycentricCoordinates(uv0, uv1, uv2, uvp);

    // 使用重心坐标插值3D点
    Vector3 point3D = interpolate3DPoint(face, bary);

    // 构建结果
    MappingResult result;
    result.point3D = point3D;
    result.pointUV = uvPoint;
    result.barycentric.face = face;
    result.barycentric.alpha = bary(0);
    result.barycentric.beta = bary(1);
    result.barycentric.gamma = bary(2);
    result.barycentric.isValid = isPointInTriangle(bary);
    result.success = result.barycentric.isValid;
    result.distanceToNearest = 0.0;

    return result;
}

std::optional<BarycentricMapper::MappingResult>
BarycentricMapper::map3DtoUV(const Vector3& point3D)
{
    if (!mesh_ || !geometry_) {
        return std::nullopt;
    }

    // 查找最近的三角形
    auto faceOpt = findNearestTriangle(point3D);
    if (!faceOpt.has_value()) {
        return std::nullopt;
    }

    Face face = faceOpt.value();

    // 获取三角形3D顶点
    auto vertices3D = getTriangleVertices3D(face);

    // 计算重心坐标
    Eigen::Vector3d p0(vertices3D[0].x, vertices3D[0].y, vertices3D[0].z);
    Eigen::Vector3d p1(vertices3D[1].x, vertices3D[1].y, vertices3D[1].z);
    Eigen::Vector3d p2(vertices3D[2].x, vertices3D[2].y, vertices3D[2].z);
    Eigen::Vector3d pp(point3D.x, point3D.y, point3D.z);

    Eigen::Vector3d bary = computeBarycentricCoordinates(p0, p1, p2, pp);

    // 使用重心坐标插值UV点
    Vector2 pointUV = interpolateUVPoint(face, bary);

    // 构建结果
    MappingResult result;
    result.point3D = point3D;
    result.pointUV = pointUV;
    result.barycentric.face = face;
    result.barycentric.alpha = bary(0);
    result.barycentric.beta = bary(1);
    result.barycentric.gamma = bary(2);
    result.barycentric.isValid = isPointInTriangle(bary);
    result.success = result.barycentric.isValid;
    result.distanceToNearest = 0.0;

    return result;
}

std::vector<BarycentricMapper::MappingResult>
BarycentricMapper::batchMapUVto3D(const std::vector<Vector2>& uvPoints)
{
    std::vector<MappingResult> results;
    results.reserve(uvPoints.size());

    for (const auto& uvPoint : uvPoints) {
        auto resultOpt = mapUVto3D(uvPoint);
        if (resultOpt.has_value()) {
            results.push_back(resultOpt.value());
        } else {
            // 映射失败，添加无效结果
            MappingResult invalid;
            invalid.success = false;
            invalid.pointUV = uvPoint;
            results.push_back(invalid);
        }
    }

    return results;
}

std::vector<BarycentricMapper::MappingResult>
BarycentricMapper::batchMap3DtoUV(const std::vector<Vector3>& points3D)
{
    std::vector<MappingResult> results;
    results.reserve(points3D.size());

    for (const auto& point3D : points3D) {
        auto resultOpt = map3DtoUV(point3D);
        if (resultOpt.has_value()) {
            results.push_back(resultOpt.value());
        } else {
            // 映射失败，添加无效结果
            MappingResult invalid;
            invalid.success = false;
            invalid.point3D = point3D;
            results.push_back(invalid);
        }
    }

    return results;
}

// ============================================================================
// 静态工具函数 - 重心坐标计算 (2D)
// ============================================================================

Eigen::Vector3d BarycentricMapper::computeBarycentricCoordinates(
    const Eigen::Vector2d& v0,
    const Eigen::Vector2d& v1,
    const Eigen::Vector2d& v2,
    const Eigen::Vector2d& point)
{
    // 使用面积法计算重心坐标
    // α = Area(point, v1, v2) / Area(v0, v1, v2)
    // β = Area(v0, point, v2) / Area(v0, v1, v2)
    // γ = Area(v0, v1, point) / Area(v0, v1, v2)

    auto signedArea = [](const Eigen::Vector2d& a,
                        const Eigen::Vector2d& b,
                        const Eigen::Vector2d& c) -> double {
        return 0.5 * ((b.x() - a.x()) * (c.y() - a.y()) -
                     (c.x() - a.x()) * (b.y() - a.y()));
    };

    double totalArea = signedArea(v0, v1, v2);

    // 检查退化三角形
    if (std::abs(totalArea) < 1e-10) {
        // 退化情况: 返回第一个顶点
        return Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    double alpha = signedArea(point, v1, v2) / totalArea;
    double beta  = signedArea(v0, point, v2) / totalArea;
    double gamma = signedArea(v0, v1, point) / totalArea;

    return Eigen::Vector3d(alpha, beta, gamma);
}

// ============================================================================
// 静态工具函数 - 重心坐标计算 (3D)
// ============================================================================

Eigen::Vector3d BarycentricMapper::computeBarycentricCoordinates(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2,
    const Eigen::Vector3d& point)
{
    // 3D三角形的重心坐标计算
    // 将点投影到三角形平面，然后使用2D方法

    // 构建三角形的局部坐标系
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    Eigen::Vector3d normal = edge1.cross(edge2);

    // 检查退化三角形
    if (normal.norm() < 1e-10) {
        return Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    normal.normalize();

    // 构建局部坐标系 (U, V轴)
    Eigen::Vector3d u = edge1.normalized();
    Eigen::Vector3d v = normal.cross(u).normalized();

    // 将3D点投影到局部2D坐标系
    auto project = [&](const Eigen::Vector3d& p) -> Eigen::Vector2d {
        Eigen::Vector3d local = p - v0;
        return Eigen::Vector2d(local.dot(u), local.dot(v));
    };

    Eigen::Vector2d p0_2d(0.0, 0.0);  // v0在原点
    Eigen::Vector2d p1_2d = project(v1);
    Eigen::Vector2d p2_2d = project(v2);
    Eigen::Vector2d pp_2d = project(point);

    // 使用2D重心坐标计算
    return computeBarycentricCoordinates(p0_2d, p1_2d, p2_2d, pp_2d);
}

// ============================================================================
// 静态工具函数 - 点在三角形内判定
// ============================================================================

bool BarycentricMapper::isPointInTriangle(
    const Eigen::Vector3d& bary,
    double epsilon)
{
    // 检查所有重心坐标是否非负 (允许epsilon容差)
    return bary(0) >= -epsilon &&
           bary(1) >= -epsilon &&
           bary(2) >= -epsilon;
}

// ============================================================================
// 插值函数
// ============================================================================

Vector3 BarycentricMapper::interpolate3DPoint(
    Face face,
    const Eigen::Vector3d& bary)
{
    auto vertices = getTriangleVertices3D(face);

    // P = α*v0 + β*v1 + γ*v2
    Vector3 result;
    result.x = bary(0) * vertices[0].x + bary(1) * vertices[1].x + bary(2) * vertices[2].x;
    result.y = bary(0) * vertices[0].y + bary(1) * vertices[1].y + bary(2) * vertices[2].y;
    result.z = bary(0) * vertices[0].z + bary(1) * vertices[1].z + bary(2) * vertices[2].z;

    return result;
}

Vector2 BarycentricMapper::interpolateUVPoint(
    Face face,
    const Eigen::Vector3d& bary)
{
    auto uvVertices = getTriangleVerticesUV(face);

    // UV = α*uv0 + β*uv1 + γ*uv2
    Vector2 result;
    result.x = bary(0) * uvVertices[0].x + bary(1) * uvVertices[1].x + bary(2) * uvVertices[2].x;
    result.y = bary(0) * uvVertices[0].y + bary(1) * uvVertices[1].y + bary(2) * uvVertices[2].y;

    return result;
}

// ============================================================================
// 三角形查找
// ============================================================================

std::optional<Face> BarycentricMapper::findUVTriangle(const Vector2& uvPoint)
{
    if (!mesh_ || uvMapping_.uvCoordinates.empty()) {
        return std::nullopt;
    }

    // 线性搜索所有三角形
    for (Face face : mesh_->faces()) {
        auto uvVertices = getTriangleVerticesUV(face);

        // 计算重心坐标
        Eigen::Vector2d uv0(uvVertices[0].x, uvVertices[0].y);
        Eigen::Vector2d uv1(uvVertices[1].x, uvVertices[1].y);
        Eigen::Vector2d uv2(uvVertices[2].x, uvVertices[2].y);
        Eigen::Vector2d uvp(uvPoint.x, uvPoint.y);

        Eigen::Vector3d bary = computeBarycentricCoordinates(uv0, uv1, uv2, uvp);

        // 检查点是否在三角形内
        if (isPointInTriangle(bary, 1e-8)) {
            return face;
        }
    }

    return std::nullopt;
}

std::optional<Face> BarycentricMapper::findNearestTriangle(const Vector3& point3D)
{
    if (!mesh_ || !geometry_) {
        return std::nullopt;
    }

    // 简化版本: 查找最近的三角形 (投影距离最小)
    Face nearestFace;
    double minDistance = std::numeric_limits<double>::max();
    bool found = false;

    for (Face face : mesh_->faces()) {
        Vector3 projection;
        double dist = distanceToTriangle(point3D, face, projection);

        if (dist < minDistance) {
            minDistance = dist;
            nearestFace = face;
            found = true;
        }
    }

    if (found) {
        return nearestFace;
    }

    return std::nullopt;
}

// ============================================================================
// 私有辅助函数
// ============================================================================

double BarycentricMapper::distanceToTriangle(
    const Vector3& point,
    Face face,
    Vector3& projection)
{
    auto vertices = getTriangleVertices3D(face);

    // 将点投影到三角形平面
    Eigen::Vector3d p(point.x, point.y, point.z);
    Eigen::Vector3d v0(vertices[0].x, vertices[0].y, vertices[0].z);
    Eigen::Vector3d v1(vertices[1].x, vertices[1].y, vertices[1].z);
    Eigen::Vector3d v2(vertices[2].x, vertices[2].y, vertices[2].z);

    // 计算平面法向量
    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    Eigen::Vector3d normal = edge1.cross(edge2);

    if (normal.norm() < 1e-10) {
        // 退化三角形
        projection = vertices[0];
        return (point - vertices[0]).norm();
    }

    normal.normalize();

    // 投影到平面
    Eigen::Vector3d v = p - v0;
    double dist = v.dot(normal);
    Eigen::Vector3d projectedPoint = p - dist * normal;

    // 计算投影点的重心坐标
    Eigen::Vector3d bary = computeBarycentricCoordinates(v0, v1, v2, projectedPoint);

    // 如果投影点在三角形内，直接返回垂直距离
    if (isPointInTriangle(bary, 1e-8)) {
        projection.x = projectedPoint.x();
        projection.y = projectedPoint.y();
        projection.z = projectedPoint.z();
        return std::abs(dist);
    }

    // 投影点在三角形外，计算到边的最近距离
    // (简化: 返回到重心的距离)
    Eigen::Vector3d center = (v0 + v1 + v2) / 3.0;
    projection.x = center.x();
    projection.y = center.y();
    projection.z = center.z();

    return (p - center).norm();
}

double BarycentricMapper::distanceToTriangleUV(
    const Vector2& point,
    Face face,
    Vector2& projection)
{
    auto uvVertices = getTriangleVerticesUV(face);

    Eigen::Vector2d p(point.x, point.y);
    Eigen::Vector2d uv0(uvVertices[0].x, uvVertices[0].y);
    Eigen::Vector2d uv1(uvVertices[1].x, uvVertices[1].y);
    Eigen::Vector2d uv2(uvVertices[2].x, uvVertices[2].y);

    // 计算重心坐标
    Eigen::Vector3d bary = computeBarycentricCoordinates(uv0, uv1, uv2, p);

    // 如果点在三角形内，距离为0
    if (isPointInTriangle(bary, 1e-8)) {
        projection = point;
        return 0.0;
    }

    // 点在三角形外，计算到重心的距离
    Eigen::Vector2d center = (uv0 + uv1 + uv2) / 3.0;
    projection.x = center.x();
    projection.y = center.y();

    return (p - center).norm();
}

void BarycentricMapper::handleBoundaryCase(MappingResult& result)
{
    // 边界情况处理 (简化版本)
    // TODO: 实现点捕捉到边和顶点的逻辑
}

std::array<Vector3, 3> BarycentricMapper::getTriangleVertices3D(Face face)
{
    std::array<Vector3, 3> vertices;
    int i = 0;

    for (Vertex v : face.adjacentVertices()) {
        vertices[i++] = geometry_->vertexPositions[v];
    }

    return vertices;
}

std::array<Vector2, 3> BarycentricMapper::getTriangleVerticesUV(Face face)
{
    std::array<Vector2, 3> uvVertices;
    int i = 0;

    for (Vertex v : face.adjacentVertices()) {
        size_t idx = v.getIndex();
        if (idx < uvMapping_.uvCoordinates.size()) {
            uvVertices[i++] = uvMapping_.uvCoordinates[idx];
        } else {
            // UV坐标缺失，使用零向量
            uvVertices[i++] = Vector2{0.0, 0.0};
        }
    }

    return uvVertices;
}

} // namespace SurfaceTextureMapping