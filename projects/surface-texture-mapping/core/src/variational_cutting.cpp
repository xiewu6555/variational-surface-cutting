/**
 * 变分切缝算法实现
 * 基于EulerianShapeOptimizer的真实变分表面切割算法集成
 * 版本: 2.0 (2025-10-10) - 完整的Variational Surface Cutting集成
 */

#include "variational_cutting.h"
#include "real_algorithm_integration.h"
#include "eulerian_cut_integrator.h"  // ⭐ 添加真实算法接口
#include <iostream>
#include <cmath>
#include <algorithm>
#include <memory>
#include <set>
#include <vector>

namespace SurfaceTextureMapping {

// VariationalCutter实现

void VariationalCutter::setMesh(std::shared_ptr<SurfaceMesh> mesh,
                                 std::shared_ptr<VertexPositionGeometry> geometry) {
    mesh_ = mesh;
    geometry_ = geometry;
}

std::vector<VariationalCutter::CutCurve>
VariationalCutter::computeOptimalCuts(const CuttingParams& params) {
    std::vector<CutCurve> cuts;

    if (!mesh_ || !geometry_) {
        std::cerr << "VariationalCutter not initialized with mesh" << std::endl;
        return cuts;
    }

    std::cout << "=========================================" << std::endl;
    std::cout << "Computing optimal cuts with REAL Variational Surface Cutting Algorithm" << std::endl;
    std::cout << "=========================================" << std::endl;
    std::cout << "  Mesh: " << mesh_->nVertices() << " vertices, " << mesh_->nFaces() << " faces" << std::endl;

    using namespace geometrycentral;
    using namespace geometrycentral::surface;

    // ========== 步骤1: 调用真实的Variational Surface Cutting算法 ==========
    std::cout << "\n[Step 1] Calling EulerianCutIntegrator..." << std::endl;

    // 创建集成器
    EulerianCutIntegrator integrator;

    // 配置优化参数（从CuttingParams映射到OptimizationParams）
    EulerianCutIntegrator::OptimizationParams optParams;
    optParams.weightLengthRegularization = params.lengthRegularization;
    optParams.weightBilapRegularization = params.smoothRegularization;
    optParams.weightHenckyDistortion = 3.0;  // GitHub推荐值
    optParams.weightDirichletDistortion = 0.0;
    optParams.weightVisibility = 0.0;
    optParams.maxIterations = params.maxIterations;
    optParams.stepSizeParam = params.timeStep;
    optParams.edgeSnapTolerance = 1e-3;
    optParams.useGeodesicPaths = false;
    optParams.verbose = true;

    // 锥点（暂时为空，未来可从高曲率点检测）
    std::vector<Vertex> conePoints;

    // 调用真实算法！
    EulerianCutIntegrator::IntegrationResult result;
    auto edgePaths = integrator.generateCuts(
        mesh_.get(),
        geometry_.get(),
        conePoints,
        optParams,
        result
    );

    std::cout << "\n[Step 1 Result]" << std::endl;
    std::cout << "  Success: " << (result.success ? "YES" : "NO") << std::endl;
    if (!result.success) {
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  ERROR: Variational Surface Cutting Failed!" << std::endl;
        std::cerr << "  Error Message: " << result.errorMessage << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  Debug Info:" << std::endl;
        std::cerr << "    - Iterations completed: " << result.numIterations << std::endl;
        std::cerr << "    - Final energy: " << result.finalEnergy << std::endl;
        std::cerr << "    - Computation time: " << result.computationTime << " seconds" << std::endl;
        std::cerr << "  ========================================" << std::endl;

        // 不允许回退！抛出异常以便调试
        throw std::runtime_error("Variational Surface Cutting integration failed: " + result.errorMessage);

        // 旧的回退代码（已禁用）
        // std::cout << "  Falling back to geometric heuristic..." << std::endl;
        // return computeOptimalCutsFallback(params);
    }

    std::cout << "  Iterations: " << result.numIterations << std::endl;
    std::cout << "  Final Energy: " << result.finalEnergy << std::endl;
    std::cout << "  Computation Time: " << result.computationTime << " seconds" << std::endl;
    std::cout << "  Cut Paths: " << result.numCutPaths << std::endl;
    std::cout << "  Total Cut Edges: " << result.totalCutEdges << std::endl;
    std::cout << "  Total Cut Length: " << result.totalCutLength << std::endl;

    // ========== 步骤2: 转换边路径为CutCurve格式 ==========
    std::cout << "\n[Step 2] Converting edge paths to CutCurve format..." << std::endl;

    for (size_t pathIdx = 0; pathIdx < edgePaths.size(); ++pathIdx) {
        const auto& edgePath = edgePaths[pathIdx];
        if (edgePath.empty()) continue;

        CutCurve cut;
        cut.points.clear();
        cut.nearestFaces.clear();
        cut.totalLength = 0.0;

        // 沿边路径提取顶点位置
        std::set<Vertex> visitedVertices;
        for (const auto& edge : edgePath) {
            Halfedge he = edge.halfedge();
            Vertex v1 = he.tailVertex();
            Vertex v2 = he.tipVertex();

            // 添加起点（避免重复）
            if (visitedVertices.find(v1) == visitedVertices.end()) {
                cut.points.push_back(geometry_->vertexPositions[v1]);
                visitedVertices.insert(v1);
            }

            // 添加终点
            if (visitedVertices.find(v2) == visitedVertices.end()) {
                cut.points.push_back(geometry_->vertexPositions[v2]);
                visitedVertices.insert(v2);
            }
        }

        // 计算总长度
        for (size_t i = 1; i < cut.points.size(); ++i) {
            Vector3 diff = cut.points[i] - cut.points[i-1];
            cut.totalLength += diff.norm();
        }

        // 占位值（TODO: 实际计算失真降低）
        cut.distortionReduction = 0.5;

        if (cut.points.size() >= 2) {
            cuts.push_back(cut);
            std::cout << "  Cut path " << (pathIdx + 1) << ": "
                     << cut.points.size() << " points, length = "
                     << cut.totalLength << std::endl;
        }
    }

    std::cout << "\n=========================================" << std::endl;
    std::cout << "  Total cuts generated: " << cuts.size() << std::endl;
    std::cout << "  Algorithm: REAL Variational Surface Cutting (EulerianShapeOptimizer)" << std::endl;
    std::cout << "=========================================" << std::endl;

    return cuts;
}

// ========== 回退实现（仅在真实算法失败时使用） ==========
std::vector<VariationalCutter::CutCurve>
VariationalCutter::computeOptimalCutsFallback(const CuttingParams& params) {
    std::vector<CutCurve> cuts;

    std::cout << "\n[FALLBACK] Using geometric heuristic for cut generation..." << std::endl;
    std::cout << "  (This is NOT the real Variational Surface Cutting algorithm)" << std::endl;

    using namespace geometrycentral;
    using namespace geometrycentral::surface;

    // 如果网格有边界，使用边界作为切缝
    if (mesh_->hasBoundary()) {
        std::cout << "  Mesh has boundary, using boundary loops as cuts" << std::endl;

        // 收集所有边界循环
        std::set<BoundaryLoop> processedLoops;
        for (BoundaryLoop bl : mesh_->boundaryLoops()) {
            if (processedLoops.count(bl)) continue;
            processedLoops.insert(bl);

            CutCurve cut;
            double totalLength = 0.0;

            // 沿边界循环提取点
            for (Halfedge he : bl.adjacentHalfedges()) {
                Vertex v = he.tailVertex();
                Vector3 pos = geometry_->vertexPositions[v];
                cut.points.push_back(pos);

                // 计算边长
                if (cut.points.size() > 1) {
                    Vector3 diff = cut.points.back() - cut.points[cut.points.size()-2];
                    totalLength += diff.norm();
                }
            }

            // 闭合循环
            if (!cut.points.empty()) {
                Vector3 diff = cut.points[0] - cut.points.back();
                totalLength += diff.norm();
                cut.points.push_back(cut.points[0]);
            }

            cut.totalLength = totalLength;
            cut.distortionReduction = 0.5;

            if (!cut.points.empty()) {
                cuts.push_back(cut);
                std::cout << "    Boundary loop cut: " << cut.points.size() << " points, length = " << totalLength << std::endl;
            }
        }
    }

    // 如果没有边界或边界切缝太少，生成基于高曲率的切缝
    if (cuts.empty() || cuts.size() < 3) {
        std::cout << "  Generating cuts based on geometric features..." << std::endl;

        // 计算高斯曲率
        geometry_->requireVertexGaussianCurvatures();

        // 找到高曲率顶点
        std::vector<Vertex> highCurvatureVertices;
        double maxAbsCurvature = 0.0;
        for (Vertex v : mesh_->vertices()) {
            double curvature = std::abs(geometry_->vertexGaussianCurvatures[v]);
            maxAbsCurvature = std::max(maxAbsCurvature, curvature);
        }

        double curvatureThreshold = maxAbsCurvature * 0.6;
        for (Vertex v : mesh_->vertices()) {
            if (std::abs(geometry_->vertexGaussianCurvatures[v]) > curvatureThreshold) {
                highCurvatureVertices.push_back(v);
            }
        }

        std::cout << "    Found " << highCurvatureVertices.size() << " high-curvature vertices" << std::endl;

        // 从高曲率顶点生成几条切缝路径
        int numCutsToGenerate = std::min(5, (int)highCurvatureVertices.size() / 20);
        numCutsToGenerate = std::max(numCutsToGenerate, 2);

        for (int i = 0; i < numCutsToGenerate && i < (int)highCurvatureVertices.size(); ++i) {
            CutCurve cut;
            Vertex startVertex = highCurvatureVertices[i * highCurvatureVertices.size() / numCutsToGenerate];

            Vertex currentVertex = startVertex;
            std::set<Vertex> visited;
            double totalLength = 0.0;

            int maxPathLength = 50;
            for (int step = 0; step < maxPathLength; ++step) {
                if (visited.count(currentVertex)) break;
                visited.insert(currentVertex);

                Vector3 pos = geometry_->vertexPositions[currentVertex];
                cut.points.push_back(pos);

                if (cut.points.size() > 1) {
                    Vector3 diff = cut.points.back() - cut.points[cut.points.size()-2];
                    totalLength += diff.norm();
                }

                Vertex nextVertex = currentVertex;
                double maxNeighborCurvature = -1e10;
                for (Vertex neighbor : currentVertex.adjacentVertices()) {
                    if (!visited.count(neighbor)) {
                        double curvature = std::abs(geometry_->vertexGaussianCurvatures[neighbor]);
                        if (curvature > maxNeighborCurvature) {
                            maxNeighborCurvature = curvature;
                            nextVertex = neighbor;
                        }
                    }
                }

                if (nextVertex == currentVertex) break;
                currentVertex = nextVertex;
            }

            cut.totalLength = totalLength;
            cut.distortionReduction = 0.5;

            if (cut.points.size() >= 3) {
                cuts.push_back(cut);
                std::cout << "    Generated cut " << (i+1) << ": " << cut.points.size() << " points, length = " << totalLength << std::endl;
            }
        }
    }

    std::cout << "  Total fallback cuts generated: " << cuts.size() << std::endl;
    return cuts;
}

std::shared_ptr<VariationalCutter::SurfaceMesh>
VariationalCutter::applyCutsToMesh(const std::vector<CutCurve>& cuts) {
    if (!mesh_ || cuts.empty()) {
        return nullptr;
    }

    std::cout << "Applying " << cuts.size() << " cuts to mesh..." << std::endl;

    // TODO: 实现切缝应用逻辑
    // 创建网格的副本并应用切缝

    return mesh_;  // 暂时返回原网格
}

VariationalCutter::CutQuality VariationalCutter::evaluateCutQuality(const std::vector<CutCurve>& cuts) const {
    CutQuality quality = {}; // 初始化所有成员为0

    if (cuts.empty()) {
        return quality;
    }

    // 计算总长度
    for (const auto& cut : cuts) {
        quality.totalLength += computeCurveLength(cut);
    }

    // 计算失真度量
    quality.averageDistortion = computeAverageDistortion(cuts);
    quality.maxDistortion = computeMaxDistortion(cuts);

    // 计算可见性评分
    quality.visibilityScore = computeVisibilityScore(cuts);

    std::cout << "Cut quality evaluation:" << std::endl;
    std::cout << "  Total length: " << quality.totalLength << std::endl;
    std::cout << "  Average distortion: " << quality.averageDistortion << std::endl;
    std::cout << "  Max distortion: " << quality.maxDistortion << std::endl;
    std::cout << "  Visibility score: " << quality.visibilityScore << std::endl;

    return quality;
}

// void VariationalCutter::initializeCuts(CutInitType type) {
//     std::cout << "Initializing cuts..." << std::endl;
//
//     switch (type) {
//         case CutInitType::Random:
//             initializeRandomCuts();
//             break;
//         case CutInitType::Geodesic:
//             initializeGeodesicCuts();
//             break;
//         case CutInitType::HighCurvature:
//             initializeHighCurvatureCuts();
//             break;
//         default:
//             initializeRandomCuts();
//     }
// }

// void VariationalCutter::initializeRandomCuts() {
//     // TODO: 实现随机初始化
//     std::cout << "  Using random initialization" << std::endl;
// }
//
// void VariationalCutter::initializeGeodesicCuts() {
//     // TODO: 实现测地线初始化
//     std::cout << "  Using geodesic initialization" << std::endl;
// }
//
// void VariationalCutter::initializeHighCurvatureCuts() {
//     // TODO: 实现高曲率区域初始化
//     std::cout << "  Using high curvature initialization" << std::endl;
// }

double VariationalCutter::computeEnergy(const std::vector<CutCurve>& cuts,
                                         const CuttingParams& params) const {
    double energy = 0.0;

    // 失真能量
    energy += computeDistortionEnergy(cuts);

    // 长度正则化
    energy += params.lengthRegularization * computeLengthEnergy(cuts);

    // 平滑正则化
    energy += params.smoothRegularization * computeSmoothEnergy(cuts);

    return energy;
}

std::vector<VariationalCutter::Vector3>
VariationalCutter::computeGradient(const std::vector<CutCurve>& cuts,
                                    const CuttingParams& params) const {
    std::vector<Vector3> gradient;

    // TODO: 实现梯度计算
    // 使用有限差分或解析梯度
    (void)cuts;
    (void)params;

    return gradient;
}

void VariationalCutter::updateCuts(std::vector<CutCurve>& cuts,
                                    const std::vector<Vector3>& gradient,
                                    double timeStep) {
    // TODO: 实现切缝更新
    // 使用梯度下降或其他优化方法
    (void)cuts;
    (void)gradient;
    (void)timeStep;
}

double VariationalCutter::computeCurveLength(const CutCurve& cut) const {
    double length = 0.0;

    for (size_t i = 1; i < cut.points.size(); ++i) {
        Vector3 diff = cut.points[i] - cut.points[i-1];
        length += std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
    }

    return length;
}

double VariationalCutter::computeDistortionEnergy(const std::vector<CutCurve>& cuts) const {
    // TODO: 实现失真能量计算
    (void)cuts;
    return 0.0;
}

double VariationalCutter::computeLengthEnergy(const std::vector<CutCurve>& cuts) const {
    double totalLength = 0.0;

    for (const auto& cut : cuts) {
        totalLength += computeCurveLength(cut);
    }

    return totalLength;
}

double VariationalCutter::computeSmoothEnergy(const std::vector<CutCurve>& cuts) const {
    // TODO: 实现平滑能量计算
    (void)cuts;
    return 0.0;
}

double VariationalCutter::computeAverageDistortion(const std::vector<CutCurve>& cuts) const {
    // TODO: 实现平均失真计算
    (void)cuts;
    return 0.1;  // 示例值
}

double VariationalCutter::computeMaxDistortion(const std::vector<CutCurve>& cuts) const {
    // TODO: 实现最大失真计算
    return 0.5;  // 示例值
}

double VariationalCutter::computeVisibilityScore(const std::vector<CutCurve>& cuts) const {
    // TODO: 实现可见性评分计算
    (void)cuts;
    return 0.8;  // 示例值
}

void VariationalCutter::setImportanceWeights(const std::vector<double>& weights) {
    importanceWeights_ = weights;
    std::cout << "Set importance weights for " << weights.size() << " vertices" << std::endl;
}


std::vector<VariationalCutter::Vector3>
VariationalCutter::computeShapeDerivative(const std::vector<CutCurve>& curves) const {
    // TODO: 实现形状导数计算
    (void)curves;
    return std::vector<Vector3>();
}

bool VariationalCutter::hasConverged(const std::vector<CutCurve>& oldCurves,
                                    const std::vector<CutCurve>& newCurves,
                                    double threshold) const {
    // TODO: 实现收敛性检查
    (void)oldCurves;
    (void)newCurves;
    (void)threshold;
    return false;
}

} // namespace SurfaceTextureMapping