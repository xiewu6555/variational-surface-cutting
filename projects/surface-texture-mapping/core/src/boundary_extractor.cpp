/**
 * Boundary Extractor Implementation
 * 边界线到网格边路径的完整转换实现
 */

#include "boundary_extractor.h"

// Core库
#include "halfedge_mesh.h"
#include "geometry.h"

#include <iostream>
#include <cmath>
#include <limits>
#include <unordered_set>
#include <queue>

namespace SurfaceTextureMapping {

/**
 * 主入口：提取边路径
 */
std::vector<std::vector<BoundaryExtractor::GC_Edge>>
BoundaryExtractor::extractEdgePaths(
    const std::vector<BoundaryLine>& boundaries,
    GC_Mesh* gcMesh,
    GC_Geometry* gcGeometry,
    Core_Mesh* coreMesh,
    Core_Geometry* coreGeometry,
    const ExtractionOptions& options,
    ExtractionResult& outResult) {

    std::vector<std::vector<GC_Edge>> result;
    outResult.success = false;

    if (options.verbose) {
        std::cout << "[BoundaryExtractor] Processing " << boundaries.size() << " boundaries" << std::endl;
    }

    try {
        double totalSnapError = 0.0;
        int numSnapped = 0;

        for (const auto& boundary : boundaries) {
            // Step 1: Snap边界点到最近的网格顶点
            double dist1, dist2;
            VertexPtr v1 = findNearestVertex(boundary.start, coreMesh, coreGeometry, dist1);
            VertexPtr v2 = findNearestVertex(boundary.end, coreMesh, coreGeometry, dist2);

            if (v1 == nullptr || v2 == nullptr) {
                if (options.verbose) {
                    std::cerr << "  Warning: Failed to snap boundary endpoints" << std::endl;
                }
                continue;
            }

            // 记录snap误差
            totalSnapError += dist1 + dist2;
            numSnapped += 2;

            outResult.maxSnapError = std::max(outResult.maxSnapError, std::max(dist1, dist2));

            // 改进：如果两端点snap到同一顶点，尝试细分边界
            if (v1 == v2) {
                // 检查边界长度，如果太短则跳过
                double boundaryLength = distance(boundary.start, boundary.end);
                if (boundaryLength < options.snapTolerance * 2.0) {
                    if (options.verbose) {
                        std::cout << "  Warning: Boundary too short (length=" << boundaryLength
                                  << "), skipping" << std::endl;
                    }
                    continue;
                }

                // 尝试使用中点
                GC_Vector3 midpoint{
                    (boundary.start.x + boundary.end.x) * 0.5,
                    (boundary.start.y + boundary.end.y) * 0.5,
                    (boundary.start.z + boundary.end.z) * 0.5
                };

                double distMid;
                VertexPtr vMid = findNearestVertex(midpoint, coreMesh, coreGeometry, distMid);

                if (vMid != nullptr && vMid != v1) {
                    // 成功找到不同的中间顶点，分成两段处理
                    // 段1: v1 -> vMid
                    auto path1 = findShortestEdgePath(v1, vMid, coreMesh, coreGeometry, options.maxPathLength);
                    auto gcPath1 = mapCoreEdgesToGC(path1, coreMesh, gcMesh);

                    // 段2: vMid -> v2 (注意v2此时等于v1，所以实际是vMid到v1的回路)
                    auto path2 = findShortestEdgePath(vMid, v1, coreMesh, coreGeometry, options.maxPathLength);
                    auto gcPath2 = mapCoreEdgesToGC(path2, coreMesh, gcMesh);

                    // 合并两段
                    if (!gcPath1.empty() && validateEdgePath(gcPath1, gcMesh)) {
                        result.push_back(gcPath1);
                        outResult.numEdgesExtracted += gcPath1.size();
                    }
                    if (!gcPath2.empty() && validateEdgePath(gcPath2, gcMesh)) {
                        result.push_back(gcPath2);
                        outResult.numEdgesExtracted += gcPath2.size();
                    }
                    outResult.numBoundariesProcessed++;
                    continue;
                } else {
                    // 中点也snap到同一顶点，边界太短，跳过
                    if (options.verbose) {
                        std::cout << "  Warning: Boundary snapped to same vertex even with midpoint, skipping" << std::endl;
                    }
                    continue;
                }
            }

            // Step 2: 找到最短边路径
            auto coreEdges = findShortestEdgePath(
                v1, v2, coreMesh, coreGeometry, options.maxPathLength
            );

            if (coreEdges.empty()) {
                if (options.verbose) {
                    std::cerr << "  Warning: No path found between vertices" << std::endl;
                }
                continue;
            }

            // Step 3: 映射core边到GC边
            auto gcEdges = mapCoreEdgesToGC(coreEdges, coreMesh, gcMesh);

            if (gcEdges.empty()) {
                if (options.verbose) {
                    std::cerr << "  Warning: Failed to map edges to GC format" << std::endl;
                }
                continue;
            }

            // Step 4: 验证路径
            if (validateEdgePath(gcEdges, gcMesh)) {
                result.push_back(gcEdges);
                outResult.numEdgesExtracted += gcEdges.size();
            } else {
                if (options.verbose) {
                    std::cerr << "  Warning: Edge path validation failed" << std::endl;
                }
            }

            outResult.numBoundariesProcessed++;
        }

        // 计算平均snap误差
        outResult.numSnappedVertices = numSnapped;
        if (numSnapped > 0) {
            outResult.averageSnapError = totalSnapError / numSnapped;
        }

        outResult.success = true;

        if (options.verbose) {
            std::cout << "[BoundaryExtractor] Successfully extracted " << result.size() << " paths" << std::endl;
            std::cout << "  Total edges: " << outResult.numEdgesExtracted << std::endl;
            std::cout << "  Avg snap error: " << outResult.averageSnapError << std::endl;
            std::cout << "  Max snap error: " << outResult.maxSnapError << std::endl;
        }

    } catch (const std::exception& e) {
        outResult.errorMessage = std::string("Extraction failed: ") + e.what();
        outResult.success = false;
    }

    return result;
}

/**
 * 找到最近的顶点
 */
VertexPtr BoundaryExtractor::findNearestVertex(
    const GC_Vector3& point,
    Core_Mesh* mesh,
    Core_Geometry* geometry,
    double& outDistance) {

    VertexPtr nearest = nullptr;
    double minDist = std::numeric_limits<double>::max();

    Vector3 targetPoint = gcToCore(point);

    for (VertexPtr v : mesh->vertices()) {
        Vector3 vPos = geometry->position(v);
        double dx = vPos.x - targetPoint.x;
        double dy = vPos.y - targetPoint.y;
        double dz = vPos.z - targetPoint.z;
        double dist = std::sqrt(dx*dx + dy*dy + dz*dz);

        if (dist < minDist) {
            minDist = dist;
            nearest = v;
        }
    }

    outDistance = minDist;
    return nearest;
}

/**
 * Dijkstra最短路径算法
 */
std::vector<EdgePtr> BoundaryExtractor::findShortestEdgePath(
    VertexPtr start,
    VertexPtr end,
    Core_Mesh* mesh,
    Core_Geometry* geometry,
    int maxSteps) {

    std::vector<EdgePtr> path;

    if (start == end) {
        return path;
    }

    // Dijkstra算法实现
    std::map<VertexPtr, double> distances;
    std::map<VertexPtr, EdgePtr> previousEdge;
    std::map<VertexPtr, VertexPtr> previousVertex;

    // 初始化
    for (VertexPtr v : mesh->vertices()) {
        distances[v] = std::numeric_limits<double>::max();
    }
    distances[start] = 0.0;

    // 优先队列
    auto cmp = [](const std::pair<double, VertexPtr>& a, const std::pair<double, VertexPtr>& b) {
        return a.first > b.first;
    };
    std::priority_queue<std::pair<double, VertexPtr>,
                       std::vector<std::pair<double, VertexPtr>>,
                       decltype(cmp)> pq(cmp);

    pq.push({0.0, start});

    std::unordered_set<VertexPtr> visited;
    int steps = 0;

    while (!pq.empty() && steps < maxSteps) {
        auto [dist, current] = pq.top();
        pq.pop();
        steps++;

        if (visited.count(current)) {
            continue;
        }
        visited.insert(current);

        if (current == end) {
            // 找到目标，重建路径
            VertexPtr v = end;
            while (v != start) {
                if (previousEdge.count(v) == 0) {
                    break;
                }
                path.push_back(previousEdge[v]);
                v = previousVertex[v];
            }
            std::reverse(path.begin(), path.end());
            break;
        }

        // 检查所有邻接顶点
        for (HalfedgePtr he : current.outgoingHalfedges()) {
            VertexPtr neighbor = he.twin().vertex();
            EdgePtr edge = he.edge();

            double edgeLength = geometry->length(edge);
            double newDist = distances[current] + edgeLength;

            if (newDist < distances[neighbor]) {
                distances[neighbor] = newDist;
                previousEdge[neighbor] = edge;
                previousVertex[neighbor] = current;
                pq.push({newDist, neighbor});
            }
        }
    }

    return path;
}

/**
 * 映射core边到GC边
 */
std::vector<BoundaryExtractor::GC_Edge>
BoundaryExtractor::mapCoreEdgesToGC(
    const std::vector<EdgePtr>& coreEdges,
    Core_Mesh* coreMesh,
    GC_Mesh* gcMesh) {

    std::vector<GC_Edge> gcEdges;

    // 构建顶点索引映射
    std::map<VertexPtr, size_t> coreVertexIndex;
    size_t idx = 0;
    for (VertexPtr v : coreMesh->vertices()) {
        coreVertexIndex[v] = idx++;
    }

    // 映射每条边
    for (EdgePtr coreEdge : coreEdges) {
        HalfedgePtr he = coreEdge.halfedge();
        VertexPtr v1 = he.vertex();
        VertexPtr v2 = he.twin().vertex();

        size_t idx1 = coreVertexIndex[v1];
        size_t idx2 = coreVertexIndex[v2];

        // 在GC网格中找到对应的边
        GC_Vertex gcV1 = gcMesh->vertex(idx1);
        GC_Vertex gcV2 = gcMesh->vertex(idx2);

        // 查找连接这两个顶点的边
        bool found = false;
        for (GC_Edge gcEdge : gcV1.adjacentEdges()) {
            auto verts = gcEdge.adjacentVertices();
            if ((verts[0] == gcV1 && verts[1] == gcV2) ||
                (verts[0] == gcV2 && verts[1] == gcV1)) {
                gcEdges.push_back(gcEdge);
                found = true;
                break;
            }
        }

        if (!found) {
            std::cerr << "[BoundaryExtractor] Warning: Could not find GC edge for core edge" << std::endl;
        }
    }

    return gcEdges;
}

/**
 * 验证边路径连通性
 */
bool BoundaryExtractor::validateEdgePath(
    const std::vector<GC_Edge>& path,
    GC_Mesh* mesh) {

    if (path.empty()) {
        return false;
    }

    // 检查每条边是否与下一条边相邻
    for (size_t i = 0; i + 1 < path.size(); ++i) {
        auto verts1 = path[i].adjacentVertices();
        auto verts2 = path[i+1].adjacentVertices();

        // 检查是否有共享顶点
        bool connected = (verts1[0] == verts2[0] ||
                         verts1[0] == verts2[1] ||
                         verts1[1] == verts2[0] ||
                         verts1[1] == verts2[1]);

        if (!connected) {
            return false;
        }
    }

    return true;
}

/**
 * 计算路径长度
 */
double BoundaryExtractor::computePathLength(
    const std::vector<GC_Edge>& path,
    GC_Geometry* geometry) {

    double totalLength = 0.0;
    for (const auto& edge : path) {
        totalLength += geometry->edgeLengths[edge];
    }
    return totalLength;
}

/**
 * Vector3转换辅助函数
 */
BoundaryExtractor::GC_Vector3 BoundaryExtractor::coreToGC(const Vector3& v) {
    return GC_Vector3{v.x, v.y, v.z};
}

Vector3 BoundaryExtractor::gcToCore(const GC_Vector3& v) {
    return Vector3{v.x, v.y, v.z};
}

} // namespace SurfaceTextureMapping
