/**
 * 网格处理模块实现
 * 提供网格预处理、修复和优化功能
 */

#include "mesh_processing.h"
#include <iostream>
#include <algorithm>
#include <unordered_set>
#include <limits>
#include <numeric>

#include "geometrycentral/surface/meshio.h"
#include "geometrycentral/surface/remeshing.h"  // geometry-central的remeshing实现

namespace SurfaceTextureMapping {

// MeshProcessor实现

bool MeshProcessor::loadMesh(const std::string& filename) {
    try {
        std::cout << "Loading mesh from: " << filename << std::endl;

        // 使用geometry-central加载网格
        auto [mesh, geometry] = geometrycentral::surface::readManifoldSurfaceMesh(filename);

        if (!mesh || !geometry) {
            std::cerr << "Failed to load mesh: invalid mesh or geometry" << std::endl;
            return false;
        }

        mesh_ = std::move(mesh);
        geometry_ = std::move(geometry);

        std::cout << "Successfully loaded mesh with "
                  << mesh_->nVertices() << " vertices, "
                  << mesh_->nFaces() << " faces, "
                  << mesh_->nEdges() << " edges" << std::endl;

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error loading mesh: " << e.what() << std::endl;
        return false;
    }
}

bool MeshProcessor::saveMesh(const std::string& filename) const {
    if (!mesh_ || !geometry_) {
        std::cerr << "No valid mesh to save" << std::endl;
        return false;
    }

    try {
        std::cout << "Saving mesh to: " << filename << std::endl;
        geometrycentral::surface::writeSurfaceMesh(*mesh_, *geometry_, filename);
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error saving mesh: " << e.what() << std::endl;
        return false;
    }
}

bool MeshProcessor::basicRepair() {
    if (!mesh_) return false;

    std::cout << "Performing basic mesh repair..." << std::endl;

    // 移除重复顶点
    removeDuplicateVertices();

    // 移除退化三角形
    removeDegenerateFaces();

    // 修复法线方向
    fixNormalOrientation();

    return true;
}

/**
*
● 找到了两个可用的remeshing库：
  1. geometry-central: remeshing.h
  2. CGAL: Polygon_mesh_processing/remesh.h
 * @param targetEdgeLength
 * @param iterations
 * @param protectBoundary
 * @return
 */
bool MeshProcessor::isotropicRemeshing(double targetEdgeLength, int iterations, bool protectBoundary) {
    using namespace geometrycentral::surface;

    if (!mesh_ || !geometry_) {
        std::cerr << "Error: No mesh loaded" << std::endl;
        return false;
    }

    std::cout << "Performing isotropic remeshing..." << std::endl;
    std::cout << "  Target edge length: " << targetEdgeLength << std::endl;
    std::cout << "  Iterations: " << iterations << std::endl;
    std::cout << "  Protect boundary: " << (protectBoundary ? "Yes" : "No") << std::endl;

    // 记录重网格化前的统计信息
    size_t originalVertices = mesh_->nVertices();
    size_t originalFaces = mesh_->nFaces();

    try {
        // 配置geometry-central的remeshing参数
        RemeshOptions options;
        options.targetEdgeLength = targetEdgeLength;
        options.maxIterations = static_cast<size_t>(iterations);
        options.curvatureAdaptation = 0.0;  // 均匀边长（不考虑曲率）
        options.minRelativeLength = 0.05;   // 最小边长为目标边长的5%
        options.smoothStyle = RemeshSmoothStyle::Circumcentric;

        // 设置边界条件
        if (protectBoundary) {
            options.boundaryCondition = RemeshBoundaryCondition::Fixed;  // 固定边界
        } else {
            options.boundaryCondition = RemeshBoundaryCondition::Free;   // 自由移动边界
        }

        // 调用geometry-central的remesh函数（会直接修改mesh_和geometry_）
        remesh(*mesh_, *geometry_, options);

        // 输出重网格化后的统计信息
        std::cout << "  Remeshing completed successfully!" << std::endl;
        std::cout << "    Vertices: " << originalVertices << " → " << mesh_->nVertices()
                  << " (+" << (mesh_->nVertices() - originalVertices) << ")" << std::endl;
        std::cout << "    Faces: " << originalFaces << " → " << mesh_->nFaces()
                  << " (+" << (mesh_->nFaces() - originalFaces) << ")" << std::endl;

        // 验证网格质量
        geometry_->requireEdgeLengths();
        double minLen = std::numeric_limits<double>::max();
        double maxLen = 0.0;
        double totalLen = 0.0;

        for (auto e : mesh_->edges()) {
            double len = geometry_->edgeLengths[e];
            minLen = std::min(minLen, len);
            maxLen = std::max(maxLen, len);
            totalLen += len;
        }

        double avgLen = totalLen / mesh_->nEdges();
        double ratio = maxLen / minLen;

        std::cout << "    Edge length statistics:" << std::endl;
        std::cout << "      Min: " << minLen << std::endl;
        std::cout << "      Max: " << maxLen << std::endl;
        std::cout << "      Avg: " << avgLen << std::endl;
        std::cout << "      Ratio (max/min): " << ratio << std::endl;

        // 检查质量是否满足Variational Cutting的要求
        if (ratio > 10.0) {
            std::cout << "    WARNING: Edge ratio still > 10.0 after remeshing!" << std::endl;
            std::cout << "             This may cause 'no interior vertices' issues." << std::endl;
            std::cout << "             Consider using a smaller target edge length." << std::endl;
        } else if (ratio > 5.0) {
            std::cout << "    Note: Edge ratio is acceptable but could be improved." << std::endl;
        } else {
            std::cout << "    ✓ Mesh quality is excellent for Variational Cutting!" << std::endl;
        }

        return true;

    } catch (const std::exception& e) {
        std::cerr << "Error during remeshing: " << e.what() << std::endl;
        return false;
    }
}

bool MeshProcessor::makeManifold() {
    if (!mesh_) return false;

    std::cout << "Making mesh manifold..." << std::endl;

    // 移除非流形顶点
    removeNonManifoldVertices();

    // 移除非流形边
    removeNonManifoldEdges();

    // 封闭小孔洞
    fillSmallHoles(10);

    return true;
}

MeshProcessor::MeshStats MeshProcessor::getMeshStats() const {
    MeshStats stats = {};

    if (!mesh_ || !geometry_) {
        return stats;
    }

    // 计算实际的网格统计信息
    stats.numVertices = mesh_->nVertices();
    stats.numFaces = mesh_->nFaces();
    stats.numEdges = mesh_->nEdges();

    // 计算边长统计
    double totalLength = 0.0;
    double minLength = std::numeric_limits<double>::max();
    double maxLength = 0.0;

    for (auto e : mesh_->edges()) {
        double length = geometry_->edgeLength(e);
        totalLength += length;
        minLength = std::min(minLength, length);
        maxLength = std::max(maxLength, length);
    }

    stats.avgEdgeLength = totalLength / mesh_->nEdges();
    stats.minEdgeLength = minLength;
    stats.maxEdgeLength = maxLength;

    // 检查流形和封闭性
    stats.isManifold = mesh_->isManifold();
    stats.isClosed = mesh_->nBoundaryLoops() == 0;

    return stats;
}

void MeshProcessor::removeDuplicateVertices() {
    // TODO: 实现移除重复顶点
    std::cout << "  Removing duplicate vertices..." << std::endl;
}

void MeshProcessor::removeDegenerateFaces() {
    // TODO: 实现移除退化三角形
    std::cout << "  Removing degenerate faces..." << std::endl;
}

void MeshProcessor::fixNormalOrientation() {
    // TODO: 实现修复法线方向
    std::cout << "  Fixing normal orientation..." << std::endl;
}

void MeshProcessor::splitLongEdges(double threshold) {
    // TODO: 实现边分割
}

void MeshProcessor::collapseShortEdges(double threshold) {
    // TODO: 实现边塌陷
}

void MeshProcessor::flipEdges() {
    // TODO: 实现边翻转以改善质量
}

void MeshProcessor::smoothVertices() {
    // TODO: 实现拉普拉斯平滑
}

void MeshProcessor::projectToOriginalSurface() {
    // TODO: 实现投影到原始表面
}

void MeshProcessor::removeNonManifoldVertices() {
    // TODO: 实现移除非流形顶点
}

void MeshProcessor::removeNonManifoldEdges() {
    // TODO: 实现移除非流形边
}

void MeshProcessor::fillSmallHoles(int maxHoleSize) {
    // TODO: 实现填充小孔洞
    std::cout << "  Filling holes smaller than " << maxHoleSize << " edges..." << std::endl;
}

double MeshProcessor::computeTargetEdgeLength() const {
    // 1. 基础有效性检查
    if (!mesh_ || !geometry_) {
        std::cerr << "Warning: No mesh loaded, using default target edge length" << std::endl;
        return 0.01;
    }

    // 2. 检查网格是否有边
    if (mesh_->nEdges() == 0) {
        std::cerr << "Warning: Mesh has no edges, using default target edge length" << std::endl;
        return 0.01;
    }

    geometry_->requireEdgeLengths();

    // 3. 收集所有边长数据，过滤退化边以增强鲁棒性
    std::vector<double> edgeLengths;
    edgeLengths.reserve(mesh_->nEdges());

    constexpr double DEGENERATE_THRESHOLD = 1e-10;
    for (auto e : mesh_->edges()) {
        double length = geometry_->edgeLengths[e];
        // 过滤退化边
        if (length > DEGENERATE_THRESHOLD) {
            edgeLengths.push_back(length);
        }
    }

    // 4. 检查是否所有边都是退化的
    if (edgeLengths.empty()) {
        std::cerr << "Warning: All edges are degenerate (length < " << DEGENERATE_THRESHOLD
                  << "), using default" << std::endl;
        return 0.01;
    }

    // 5. 排序以计算统计量
    std::sort(edgeLengths.begin(), edgeLengths.end());

    // 6. 正确计算中位数（处理偶数和奇数情况）
    size_t n = edgeLengths.size();
    double medianLength;
    if (n % 2 == 0) {
        // 偶数个元素：取中间两个值的平均
        medianLength = (edgeLengths[n / 2 - 1] + edgeLengths[n / 2]) / 2.0;
    } else {
        // 奇数个元素：取正中间的值
        medianLength = edgeLengths[n / 2];
    }

    // 7. 计算其他统计量用于调试
    double minLength = edgeLengths.front();
    double maxLength = edgeLengths.back();
    double avgLength = std::accumulate(edgeLengths.begin(), edgeLengths.end(), 0.0) / n;

    // 8. 合理性边界检查和限制
    // 对于Variational Surface Cutting的隐式边界方法，需要非常细的网格分辨率
    // 以确保patches有足够的内部顶点进行Yamabe问题求解
    // ⭐ 平衡性能和质量：避免网格过密导致计算量爆炸
    constexpr double MIN_EDGE_LENGTH = 0.005;  // 最小边长（避免过度细分）
    constexpr double MAX_EDGE_LENGTH = 0.05;   // 最大边长（确保足够分辨率）

    // 根据当前网格规模自适应调整目标边长
    // 如果网格已经很大，使用较保守的细分
    double adaptiveFactor = 1.0;
    if (mesh_->nVertices() > 10000) {
        adaptiveFactor = 1.5;  // 对大网格使用较大的边长
        std::cout << "  Note: Large mesh detected, using conservative remeshing" << std::endl;
    } else if (mesh_->nVertices() < 1000) {
        adaptiveFactor = 0.5;  // 对小网格使用较小的边长
        std::cout << "  Note: Small mesh detected, using aggressive remeshing" << std::endl;
    }

    // 使用中位数的50%作为基准，乘以自适应因子
    double targetEdgeLength = medianLength * 0.5 * adaptiveFactor;
    double clampedMedian = std::max(MIN_EDGE_LENGTH, std::min(MAX_EDGE_LENGTH, targetEdgeLength));

    // 9. 输出统计信息
    std::cout << "Auto-computed target edge length statistics:" << std::endl;
    std::cout << "  Min edge: " << minLength << std::endl;
    std::cout << "  Max edge: " << maxLength << std::endl;
    std::cout << "  Avg edge: " << avgLength << std::endl;
    std::cout << "  Median edge (raw): " << medianLength << std::endl;

    if (medianLength != clampedMedian) {
        std::cout << "  WARNING: Median clamped to valid range ["
                  << MIN_EDGE_LENGTH << ", " << MAX_EDGE_LENGTH << "]" << std::endl;
        std::cout << "  Clamped median: " << clampedMedian << std::endl;
    }

    std::cout << "  Recommended target: " << clampedMedian << std::endl;

    return clampedMedian;
}

} // namespace SurfaceTextureMapping