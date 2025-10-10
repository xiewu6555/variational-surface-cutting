// UV到3D映射实现
// 使用重心坐标实现精确的UV空间到3D曲面的映射

#include "../include/surface_filling.h"
#include <geometrycentral/surface/halfedge_mesh.h>
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <unordered_map>
#include <iostream>

namespace SurfaceTextureMapping {

using namespace geometrycentral;
using namespace geometrycentral::surface;

// UV到3D映射器类
class UVTo3DMapper {
public:
    struct Triangle2D {
        Vector2 v0, v1, v2;
        size_t faceIndex;  // 对应的3D面索引
    };

    struct Triangle3D {
        Vector3 v0, v1, v2;
    };

    // 计算点在三角形中的重心坐标
    static Vector3 ComputeBarycentricCoordinates(const Vector2& p, const Triangle2D& tri) {
        Vector2 v0 = tri.v2 - tri.v0;
        Vector2 v1 = tri.v1 - tri.v0;
        Vector2 v2 = p - tri.v0;

        double dot00 = v0.x * v0.x + v0.y * v0.y;
        double dot01 = v0.x * v1.x + v0.y * v1.y;
        double dot02 = v0.x * v2.x + v0.y * v2.y;
        double dot11 = v1.x * v1.x + v1.y * v1.y;
        double dot12 = v1.x * v2.x + v1.y * v2.y;

        double invDenom = 1.0 / (dot00 * dot11 - dot01 * dot01);
        double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
        double v = (dot00 * dot12 - dot01 * dot02) * invDenom;
        double w = 1.0 - u - v;

        return Vector3{w, v, u};  // 返回重心坐标(w, v, u)对应(v0, v1, v2)
    }

    // 检查点是否在三角形内
    static bool IsPointInTriangle(const Vector2& p, const Triangle2D& tri) {
        Vector3 bary = ComputeBarycentricCoordinates(p, tri);
        return bary.x >= -1e-6 && bary.y >= -1e-6 && bary.z >= -1e-6;
    }

    // 使用重心坐标插值3D位置
    static Vector3 InterpolatePosition(const Vector3& baryCoords, const Triangle3D& tri3D) {
        return baryCoords.x * tri3D.v0 +
               baryCoords.y * tri3D.v1 +
               baryCoords.z * tri3D.v2;
    }

    // 构建UV三角形索引（用于快速查找）
    class UVTriangleIndex {
    public:
        struct GridCell {
            std::vector<size_t> triangles;
        };

        UVTriangleIndex(const std::vector<Triangle2D>& triangles, double gridSize = 0.1)
            : m_gridSize(gridSize) {

            // 计算边界框
            m_minX = m_minY = std::numeric_limits<double>::max();
            m_maxX = m_maxY = std::numeric_limits<double>::lowest();

            for (const auto& tri : triangles) {
                UpdateBounds(tri.v0);
                UpdateBounds(tri.v1);
                UpdateBounds(tri.v2);
            }

            // 创建网格
            m_gridWidth = static_cast<int>((m_maxX - m_minX) / gridSize) + 1;
            m_gridHeight = static_cast<int>((m_maxY - m_minY) / gridSize) + 1;
            m_grid.resize(m_gridWidth * m_gridHeight);

            // 将三角形添加到网格单元
            for (size_t i = 0; i < triangles.size(); i++) {
                AddTriangleToGrid(i, triangles[i]);
            }
        }

        // 查找包含给定点的三角形
        std::optional<size_t> FindTriangleContainingPoint(
            const Vector2& point,
            const std::vector<Triangle2D>& triangles) const {

            int gridX = static_cast<int>((point.x - m_minX) / m_gridSize);
            int gridY = static_cast<int>((point.y - m_minY) / m_gridSize);

            // 检查3x3邻域（处理边界情况）
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    int x = gridX + dx;
                    int y = gridY + dy;

                    if (x >= 0 && x < m_gridWidth && y >= 0 && y < m_gridHeight) {
                        const auto& cell = m_grid[y * m_gridWidth + x];

                        for (size_t triIdx : cell.triangles) {
                            if (IsPointInTriangle(point, triangles[triIdx])) {
                                return triIdx;
                            }
                        }
                    }
                }
            }

            return std::nullopt;
        }

    private:
        void UpdateBounds(const Vector2& p) {
            m_minX = std::min(m_minX, p.x);
            m_maxX = std::max(m_maxX, p.x);
            m_minY = std::min(m_minY, p.y);
            m_maxY = std::max(m_maxY, p.y);
        }

        void AddTriangleToGrid(size_t triIndex, const Triangle2D& tri) {
            // 计算三角形的边界框
            double triMinX = std::min({tri.v0.x, tri.v1.x, tri.v2.x});
            double triMaxX = std::max({tri.v0.x, tri.v1.x, tri.v2.x});
            double triMinY = std::min({tri.v0.y, tri.v1.y, tri.v2.y});
            double triMaxY = std::max({tri.v0.y, tri.v1.y, tri.v2.y});

            int minGridX = std::max(0, static_cast<int>((triMinX - m_minX) / m_gridSize));
            int maxGridX = std::min(m_gridWidth - 1, static_cast<int>((triMaxX - m_minX) / m_gridSize));
            int minGridY = std::max(0, static_cast<int>((triMinY - m_minY) / m_gridSize));
            int maxGridY = std::min(m_gridHeight - 1, static_cast<int>((triMaxY - m_minY) / m_gridSize));

            // 将三角形添加到所有重叠的网格单元
            for (int y = minGridY; y <= maxGridY; y++) {
                for (int x = minGridX; x <= maxGridX; x++) {
                    m_grid[y * m_gridWidth + x].triangles.push_back(triIndex);
                }
            }
        }

        double m_gridSize;
        double m_minX, m_maxX, m_minY, m_maxY;
        int m_gridWidth, m_gridHeight;
        std::vector<GridCell> m_grid;
    };

    // 主要的映射类
    class Mapper {
    public:
        Mapper(HalfedgeMesh* mesh,
               VertexPositionGeometry* geometry,
               const VertexData<Vector2>& uvCoordinates)
            : m_mesh(mesh), m_geometry(geometry), m_uvCoords(uvCoordinates) {

            BuildTriangleData();
            m_uvIndex = std::make_unique<UVTriangleIndex>(m_uvTriangles);
        }

        // 将UV空间的点映射到3D空间
        std::optional<Vector3> MapUVTo3D(const Vector2& uvPoint) const {
            auto triIdx = m_uvIndex->FindTriangleContainingPoint(uvPoint, m_uvTriangles);

            if (!triIdx.has_value()) {
                return std::nullopt;  // 点不在任何UV三角形内
            }

            const Triangle2D& uvTri = m_uvTriangles[*triIdx];
            const Triangle3D& tri3D = m_3DTriangles[*triIdx];

            Vector3 baryCoords = ComputeBarycentricCoordinates(uvPoint, uvTri);
            return InterpolatePosition(baryCoords, tri3D);
        }

        // 将UV路径映射到3D路径
        std::vector<Vector3> MapUVPathTo3D(const std::vector<Vector2>& uvPath) const {
            std::vector<Vector3> path3D;

            for (const auto& uvPoint : uvPath) {
                auto pos3D = MapUVTo3D(uvPoint);
                if (pos3D.has_value()) {
                    path3D.push_back(*pos3D);
                } else {
                    // 处理不在UV域内的点
                    if (!path3D.empty()) {
                        // 尝试找到最近的有效点
                        auto nearestPoint = FindNearestValidUVPoint(uvPoint);
                        if (nearestPoint.has_value()) {
                            auto nearestPos3D = MapUVTo3D(*nearestPoint);
                            if (nearestPos3D.has_value()) {
                                path3D.push_back(*nearestPos3D);
                            }
                        }
                    }
                }
            }

            return path3D;
        }

        // 批量映射多条路径
        std::vector<std::vector<Vector3>> MapMultipleUVPathsTo3D(
            const std::vector<std::vector<Vector2>>& uvPaths) const {

            std::vector<std::vector<Vector3>> paths3D;

            for (const auto& uvPath : uvPaths) {
                auto path3D = MapUVPathTo3D(uvPath);
                if (!path3D.empty()) {
                    paths3D.push_back(path3D);
                }
            }

            return paths3D;
        }

    private:
        void BuildTriangleData() {
            size_t faceIndex = 0;

            for (Face f : m_mesh->faces()) {
                std::vector<Vertex> vertices;
                std::vector<Vector3> positions3D;
                std::vector<Vector2> positionsUV;

                for (Vertex v : f.adjacentVertices()) {
                    vertices.push_back(v);
                    positions3D.push_back(m_geometry->vertexPositions[v]);
                    positionsUV.push_back(m_uvCoords[v]);
                }

                if (vertices.size() == 3) {  // 只处理三角形
                    Triangle2D uvTri;
                    uvTri.v0 = positionsUV[0];
                    uvTri.v1 = positionsUV[1];
                    uvTri.v2 = positionsUV[2];
                    uvTri.faceIndex = faceIndex;

                    Triangle3D tri3D;
                    tri3D.v0 = positions3D[0];
                    tri3D.v1 = positions3D[1];
                    tri3D.v2 = positions3D[2];

                    m_uvTriangles.push_back(uvTri);
                    m_3DTriangles.push_back(tri3D);
                }

                faceIndex++;
            }

            std::cout << "[UVTo3DMapper] 构建了 " << m_uvTriangles.size()
                      << " 个三角形的映射数据" << std::endl;
        }

        // 找到最近的有效UV点（用于处理边界情况）
        std::optional<Vector2> FindNearestValidUVPoint(const Vector2& uvPoint) const {
            double minDist = std::numeric_limits<double>::max();
            std::optional<Vector2> nearest;

            for (const auto& tri : m_uvTriangles) {
                // 检查三角形的三个顶点
                std::array<Vector2, 3> verts = {tri.v0, tri.v1, tri.v2};

                for (const auto& v : verts) {
                    double dist = (v - uvPoint).norm();
                    if (dist < minDist) {
                        minDist = dist;
                        nearest = v;
                    }
                }

                // 也可以检查边上的最近点
                for (int i = 0; i < 3; i++) {
                    Vector2 edgeStart = verts[i];
                    Vector2 edgeEnd = verts[(i + 1) % 3];

                    Vector2 closestOnEdge = ClosestPointOnLineSegment(
                        uvPoint, edgeStart, edgeEnd);

                    double dist = (closestOnEdge - uvPoint).norm();
                    if (dist < minDist && IsPointInTriangle(closestOnEdge, tri)) {
                        minDist = dist;
                        nearest = closestOnEdge;
                    }
                }
            }

            return nearest;
        }

        // 计算线段上最近的点
        static Vector2 ClosestPointOnLineSegment(
            const Vector2& point,
            const Vector2& lineStart,
            const Vector2& lineEnd) {

            Vector2 lineVec = lineEnd - lineStart;
            double lineLength2 = lineVec.x * lineVec.x + lineVec.y * lineVec.y;

            if (lineLength2 < 1e-10) {
                return lineStart;  // 退化为点
            }

            double t = ((point - lineStart).x * lineVec.x +
                       (point - lineStart).y * lineVec.y) / lineLength2;
            t = std::max(0.0, std::min(1.0, t));

            return lineStart + t * lineVec;
        }

        HalfedgeMesh* m_mesh;
        VertexPositionGeometry* m_geometry;
        VertexData<Vector2> m_uvCoords;

        std::vector<Triangle2D> m_uvTriangles;
        std::vector<Triangle3D> m_3DTriangles;
        std::unique_ptr<UVTriangleIndex> m_uvIndex;
    };
};

// SurfaceFiller类的mapPathsTo3D实现
std::vector<std::vector<Vector3>> SurfaceFiller::mapPathsTo3D(
    const std::vector<std::vector<Vector2>>& uvPaths) const {

    if (!mesh_ || !geometry_) {
        std::cerr << "[SurfaceFiller] 错误：未设置网格或几何数据" << std::endl;
        return {};
    }

    // 从UV映射中提取顶点UV坐标
    VertexData<Vector2> uvCoords(*mesh_);

    // 假设uvMapping_中存储了UV坐标
    // 这里需要根据实际的UVMapping结构来提取数据
    size_t vertexIdx = 0;
    for (Vertex v : mesh_->vertices()) {
        if (vertexIdx < uvMapping_.uvCoordinates.size()) {
            uvCoords[v] = Vector2{
                uvMapping_.uvCoordinates[vertexIdx].x,
                uvMapping_.uvCoordinates[vertexIdx].y
            };
        }
        vertexIdx++;
    }

    // 创建映射器
    UVTo3DMapper::Mapper mapper(mesh_.get(), geometry_.get(), uvCoords);

    // 映射所有路径
    return mapper.MapMultipleUVPathsTo3D(uvPaths);
}

// 导出接口供外部使用
namespace UVTo3DMapping {

std::vector<Vector3> MapSinglePathTo3D(
    HalfedgeMesh* mesh,
    VertexPositionGeometry* geometry,
    const VertexData<Vector2>& uvCoordinates,
    const std::vector<Vector2>& uvPath) {

    UVTo3DMapper::Mapper mapper(mesh, geometry, uvCoordinates);
    return mapper.MapUVPathTo3D(uvPath);
}

std::optional<Vector3> MapSinglePointTo3D(
    HalfedgeMesh* mesh,
    VertexPositionGeometry* geometry,
    const VertexData<Vector2>& uvCoordinates,
    const Vector2& uvPoint) {

    UVTo3DMapper::Mapper mapper(mesh, geometry, uvCoordinates);
    return mapper.MapUVTo3D(uvPoint);
}

} // namespace UVTo3DMapping

} // namespace SurfaceTextureMapping