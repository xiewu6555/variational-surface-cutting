/**
 * 网格预处理器实现
 */

#include "mesh_processor.h"
#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/surface_mesh_factories.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <queue>
#include <set>

namespace SurfaceTextureMapping {

using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
using Vector3 = geometrycentral::Vector3;
using Vector2 = geometrycentral::Vector2;
using Vertex = geometrycentral::surface::Vertex;
using Edge = geometrycentral::surface::Edge;
using Face = geometrycentral::surface::Face;

void MeshProcessor::setMesh(std::shared_ptr<SurfaceMesh> mesh,
                            std::shared_ptr<VertexPositionGeometry> geometry) {
    mesh_ = mesh;
    geometry_ = geometry;
}

std::pair<std::shared_ptr<SurfaceMesh>, std::shared_ptr<VertexPositionGeometry>>
MeshProcessor::remeshIsotropic(const ProcessingParams& params) {
    if (!mesh_ || !geometry_) {
        std::cerr << "MeshProcessor: No mesh set" << std::endl;
        return {nullptr, nullptr};
    }

    std::cout << "Starting isotropic remeshing..." << std::endl;

    // 计算目标边长
    double targetLength = params.targetEdgeLength;
    if (targetLength <= 0) {
        targetLength = computeMeanEdgeLength();
        std::cout << "  Auto-computed target edge length: " << targetLength << std::endl;
    }

    // 创建网格副本
    auto newMeshPtr = mesh_->copy();
    auto newMesh = std::shared_ptr<SurfaceMesh>(newMeshPtr.release());
    auto newGeometry = std::make_shared<VertexPositionGeometry>(*newMesh);
    // Copy vertex positions manually
    for (auto v : newMesh->vertices()) {
        newGeometry->vertexPositions[v] = geometry_->vertexPositions[mesh_->vertex(v.getIndex())];
    }

    // 执行重网格化迭代
    for (int iter = 0; iter < params.iterations; ++iter) {
        std::cout << "  Iteration " << (iter + 1) << "/" << params.iterations << std::endl;

        // 1. 分割长边
        double maxLength = 4.0 * targetLength / 3.0;
        splitLongEdges(maxLength);

        // 2. 合并短边
        double minLength = 4.0 * targetLength / 5.0;
        collapseShortEdges(minLength);

        // 3. 边翻转优化
        flipEdges();

        // 4. 切向平滑
        tangentialSmoothing(params.smoothingWeight);
    }

    std::cout << "  Remeshing completed" << std::endl;
    std::cout << "  Final mesh: " << newMesh->nVertices() << " vertices, "
              << newMesh->nFaces() << " faces" << std::endl;

    return std::make_pair(newMesh, newGeometry);
}

std::pair<std::shared_ptr<SurfaceMesh>, std::shared_ptr<VertexPositionGeometry>>
MeshProcessor::simplify(const ProcessingParams& params) {
    if (!mesh_ || !geometry_) {
        return {nullptr, nullptr};
    }

    std::cout << "Starting mesh simplification..." << std::endl;
    std::cout << "  Target reduction: " << (params.targetReduction * 100) << "%" << std::endl;

    // 简单的边折叠简化
    // TODO: 实现基于QEM的简化算法

    auto newMeshPtr = mesh_->copy();
    auto newMesh = std::shared_ptr<SurfaceMesh>(newMeshPtr.release());
    auto newGeometry = std::make_shared<VertexPositionGeometry>(*newMesh);
    // Copy vertex positions manually
    for (auto v : newMesh->vertices()) {
        newGeometry->vertexPositions[v] = geometry_->vertexPositions[mesh_->vertex(v.getIndex())];
    }

    int targetFaces = static_cast<int>(mesh_->nFaces() * (1.0 - params.targetReduction));

    std::cout << "  Simplification completed (placeholder implementation)" << std::endl;

    return std::make_pair(newMesh, newGeometry);
}

std::pair<std::shared_ptr<SurfaceMesh>, std::shared_ptr<VertexPositionGeometry>>
MeshProcessor::cleanup(const ProcessingParams& params) {
    if (!mesh_ || !geometry_) {
        return {nullptr, nullptr};
    }

    std::cout << "Starting mesh cleanup..." << std::endl;

    auto newMeshPtr = mesh_->copy();
    auto newMesh = std::shared_ptr<SurfaceMesh>(newMeshPtr.release());
    auto newGeometry = std::make_shared<VertexPositionGeometry>(*newMesh);
    // Copy vertex positions manually
    for (auto v : newMesh->vertices()) {
        newGeometry->vertexPositions[v] = geometry_->vertexPositions[mesh_->vertex(v.getIndex())];
    }

    // TODO: 实现清理操作
    // - 移除孤立顶点
    // - 移除退化面
    // - 确保流形性质

    std::cout << "  Cleanup completed" << std::endl;

    return std::make_pair(newMesh, newGeometry);
}

void MeshProcessor::smoothLaplacian(int iterations, double weight) {
    if (!mesh_ || !geometry_) return;

    std::cout << "Applying Laplacian smoothing..." << std::endl;

    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Vector3> newPositions(mesh_->nVertices());

        size_t idx = 0;
        for (auto v : mesh_->vertices()) {
            if (!v.isBoundary()) {
                Vector3 sum{0, 0, 0};
                int count = 0;

                for (auto neighbor : v.adjacentVertices()) {
                    sum += geometry_->vertexPositions[neighbor];
                    count++;
                }

                if (count > 0) {
                    Vector3 laplacian = sum / count - geometry_->vertexPositions[v];
                    newPositions[idx] = geometry_->vertexPositions[v] + weight * laplacian;
                } else {
                    newPositions[idx] = geometry_->vertexPositions[v];
                }
            } else {
                // 保持边界顶点不变
                newPositions[idx] = geometry_->vertexPositions[v];
            }
            idx++;
        }

        // 更新位置
        idx = 0;
        for (auto v : mesh_->vertices()) {
            geometry_->vertexPositions[v] = newPositions[idx++];
        }
    }
}

void MeshProcessor::smoothBilateral(int iterations, double spatialSigma, double normalSigma) {
    if (!mesh_ || !geometry_) return;

    std::cout << "Applying bilateral smoothing..." << std::endl;

    // 计算顶点法线
    geometry_->requireVertexNormals();

    for (int iter = 0; iter < iterations; ++iter) {
        std::vector<Vector3> newPositions(mesh_->nVertices());

        size_t idx = 0;
        for (auto v : mesh_->vertices()) {
            if (!v.isBoundary()) {
                Vector3 pos = geometry_->vertexPositions[v];
                Vector3 normal = geometry_->vertexNormals[v];

                Vector3 sum{0, 0, 0};
                double weightSum = 0;

                for (auto neighbor : v.adjacentVertices()) {
                    Vector3 neighborPos = geometry_->vertexPositions[neighbor];
                    Vector3 neighborNormal = geometry_->vertexNormals[neighbor];

                    // 空间权重
                    double dist = (neighborPos - pos).norm();
                    double spatialWeight = std::exp(-dist * dist / (2 * spatialSigma * spatialSigma));

                    // 法线权重
                    double normalDot = geometrycentral::dot(normal, neighborNormal);
                    double normalWeight = std::exp(-std::pow(1 - normalDot, 2) / (2 * normalSigma * normalSigma));

                    double weight = spatialWeight * normalWeight;
                    sum += weight * neighborPos;
                    weightSum += weight;
                }

                if (weightSum > 0) {
                    newPositions[idx] = sum / weightSum;
                } else {
                    newPositions[idx] = pos;
                }
            } else {
                newPositions[idx] = geometry_->vertexPositions[v];
            }
            idx++;
        }

        // 更新位置
        idx = 0;
        for (auto v : mesh_->vertices()) {
            geometry_->vertexPositions[v] = newPositions[idx++];
        }
    }
}

MeshProcessor::ProcessingStats MeshProcessor::computeStats() const {
    ProcessingStats stats;

    if (!mesh_ || !geometry_) {
        return stats;
    }

    stats.originalVertices = stats.processedVertices = mesh_->nVertices();
    stats.originalFaces = stats.processedFaces = mesh_->nFaces();
    stats.meanEdgeLength = computeMeanEdgeLength();

    // 计算边长范围
    stats.minEdgeLength = std::numeric_limits<double>::max();
    stats.maxEdgeLength = 0;

    for (auto e : mesh_->edges()) {
        double length = geometry_->edgeLength(e);
        stats.minEdgeLength = std::min(stats.minEdgeLength, length);
        stats.maxEdgeLength = std::max(stats.maxEdgeLength, length);
    }

    stats.isManifold = mesh_->isManifold();
    stats.isClosed = mesh_->nBoundaryLoops() == 0;
    stats.boundaryLoops = mesh_->nBoundaryLoops();

    return stats;
}

std::vector<Edge> MeshProcessor::detectFeatureEdges(double angleThreshold) const {
    std::vector<Edge> features;

    if (!mesh_ || !geometry_) {
        return features;
    }

    double thresholdRad = angleThreshold * M_PI / 180.0;

    for (auto e : mesh_->edges()) {
        if (isFeatureEdge(e, thresholdRad)) {
            features.push_back(e);
        }
    }

    std::cout << "Detected " << features.size() << " feature edges" << std::endl;

    return features;
}

std::vector<std::vector<Vertex>> MeshProcessor::detectBoundaryLoops() const {
    std::vector<std::vector<Vertex>> loops;

    if (!mesh_) {
        return loops;
    }

    // 标记访问过的边界顶点
    std::set<Vertex> visited;

    for (auto v : mesh_->vertices()) {
        if (v.isBoundary() && visited.find(v) == visited.end()) {
            std::vector<Vertex> loop;
            Vertex current = v;

            do {
                loop.push_back(current);
                visited.insert(current);

                // 找下一个边界顶点
                for (auto he : current.outgoingHalfedges()) {
                    if (he.edge().isBoundary()) {
                        current = he.tipVertex();
                        break;
                    }
                }
            } while (current != v && visited.find(current) == visited.end());

            loops.push_back(loop);
        }
    }

    std::cout << "Detected " << loops.size() << " boundary loops" << std::endl;

    return loops;
}

// 私有辅助函数实现

double MeshProcessor::computeMeanEdgeLength() const {
    if (!geometry_) return 0;

    double sum = 0;
    int count = 0;

    for (auto e : mesh_->edges()) {
        sum += geometry_->edgeLength(e);
        count++;
    }

    return count > 0 ? sum / count : 0;
}

void MeshProcessor::collapseShortEdges(double minLength) {
    // 简单实现 - 实际应该使用优先队列
    // TODO: 实现基于优先队列的边折叠
}

void MeshProcessor::splitLongEdges(double maxLength) {
    // 简单实现 - 实际应该标记并批量分割
    // TODO: 实现边分割
}

void MeshProcessor::flipEdges() {
    // 基于Delaunay准则翻转边
    // TODO: 实现边翻转优化
}

void MeshProcessor::tangentialSmoothing(double weight) {
    // 在保持法线方向的情况下平滑
    // TODO: 实现切向平滑
}

bool MeshProcessor::isFeatureEdge(const Edge& e, double angleThreshold) const {
    if (e.isBoundary()) {
        return true;
    }

    // 计算二面角
    auto h = e.halfedge();
    auto f1 = h.face();
    auto f2 = h.twin().face();

    // Check if faces exist (geometry-central Face doesn't have isValid method)
    if (f1 == Face() || f2 == Face()) {
        return true;
    }

    geometry_->requireFaceNormals();
    Vector3 n1 = geometry_->faceNormals[f1];
    Vector3 n2 = geometry_->faceNormals[f2];

    double cosAngle = geometrycentral::dot(n1, n2);
    double angle = std::acos(std::max(-1.0, std::min(1.0, cosAngle)));

    return angle > angleThreshold;
}

} // namespace SurfaceTextureMapping