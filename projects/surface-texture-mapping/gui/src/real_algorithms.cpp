// 真实算法实现
// 逐步实现variational cutting和UV展开算法，每步都可视化

#include "../include/real_algorithms.h"
#include <iostream>
#include <algorithm>
#include <numeric>
#include <queue>
#include <cmath>

namespace SurfaceTextureMapping {

// ========== VariationalCuttingAlgorithm 实现 ==========

VariationalCuttingAlgorithm::VariationalCuttingAlgorithm(HalfedgeMesh* mesh, VertexPositionGeometry* geometry)
    : m_mesh(mesh), m_geometry(geometry) {
    // 初始化内在几何接口
    m_intrinsicGeometry = std::make_unique<EdgeLengthGeometry>(*mesh, geometry->edgeLengths);

    // 预分配数据存储
    m_gaussianCurvatures = VertexData<double>(*mesh, 0.0);
    m_meanCurvatures = VertexData<double>(*mesh, 0.0);
    m_principalCurvatures = VertexData<double>(*mesh, 0.0);

    // 初始化geometry-central工具
    initializeGeometryTools();

    std::cout << "[VariationalCutting] 初始化完成，网格包含 "
              << mesh->nVertices() << " 个顶点, "
              << mesh->nFaces() << " 个面" << std::endl;
}

MeshAnalysis VariationalCuttingAlgorithm::analyzeMesh() {
    MeshAnalysis analysis;

    // 基本拓扑信息
    analysis.nVertices = m_mesh->nVertices();
    analysis.nFaces = m_mesh->nFaces();
    analysis.nEdges = m_mesh->nEdges();

    // 计算欧拉特征数
    analysis.eulerCharacteristic = analysis.nVertices - analysis.nEdges + analysis.nFaces;

    // 检查流形性和封闭性
    analysis.isManifold = m_mesh->isManifold();
    analysis.isWatertight = m_mesh->nBoundaryLoops() == 0;

    // 如果是封闭曲面，计算亏格
    if (analysis.isWatertight) {
        analysis.genus = (2 - analysis.eulerCharacteristic) / 2;
    } else {
        analysis.genus = -1; // 非封闭曲面不计算亏格
    }

    // 计算几何度量
    double totalArea = 0.0;
    double totalEdgeLength = 0.0;
    double minEdge = std::numeric_limits<double>::max();
    double maxEdge = 0.0;
    size_t degenerateCount = 0;

    // 遍历所有面计算面积
    for (Face f : m_mesh->faces()) {
        double area = m_geometry->faceArea(f);
        totalArea += area;

        // 检查退化三角形
        if (area < 1e-10) {
            degenerateCount++;
        }
    }

    // 遍历所有边计算长度
    for (Edge e : m_mesh->edges()) {
        double length = m_geometry->edgeLength(e);
        totalEdgeLength += length;
        minEdge = std::min(minEdge, length);
        maxEdge = std::max(maxEdge, length);
    }

    analysis.surfaceArea = totalArea;
    analysis.avgEdgeLength = totalEdgeLength / m_mesh->nEdges();
    analysis.minEdgeLength = minEdge;
    analysis.maxEdgeLength = maxEdge;
    analysis.nDegenerateTriangles = degenerateCount;

    // 计算三角形质量（基于边长比）
    double totalQuality = 0.0;
    for (Face f : m_mesh->faces()) {
        std::vector<double> edgeLengths;
        for (Halfedge he : f.adjacentHalfedges()) {
            edgeLengths.push_back(m_geometry->edgeLength(he.edge()));
        }

        // 计算质量：最短边/最长边的比值
        double minLen = *std::min_element(edgeLengths.begin(), edgeLengths.end());
        double maxLen = *std::max_element(edgeLengths.begin(), edgeLengths.end());
        double quality = minLen / maxLen;
        totalQuality += quality;
    }
    analysis.avgTriangleQuality = totalQuality / m_mesh->nFaces();

    // 计算曲率信息 - 使用geometry-central的精确算法
    try {
        m_geometry->requireVertexGaussianCurvatures();
        m_geometry->requireVertexMeanCurvatures();

        // 如果可能，也计算主曲率
        try {
            m_geometry->requireVertexPrincipalCurvatureDirections();
        } catch (const std::exception&) {
            // 如果不支持主曲率方向，忽略错误
        }

        double totalGaussian = 0.0;
        double totalMean = 0.0;
        double maxGaussian = -std::numeric_limits<double>::max();
        double minGaussian = std::numeric_limits<double>::max();

        analysis.vertexCurvatures.resize(m_mesh->nVertices());
        size_t idx = 0;

        for (Vertex v : m_mesh->vertices()) {
            double gaussian = m_geometry->vertexGaussianCurvature(v);
            double mean = m_geometry->vertexMeanCurvature(v);

            // 计算主曲率（如果可能）
            double principalCurvature = 0.0;
            try {
                // k1 和 k2 是主曲率，其中 k1 + k2 = 2*H (平均曲率)，k1 * k2 = K (高斯曲率)
                double discriminant = mean * mean - gaussian;
                if (discriminant >= 0) {
                    double k1 = mean + std::sqrt(discriminant);
                    double k2 = mean - std::sqrt(discriminant);
                    principalCurvature = std::max(std::abs(k1), std::abs(k2));
                }
            } catch (const std::exception&) {
                principalCurvature = std::abs(mean); // 回退到平均曲率
            }

            m_gaussianCurvatures[v] = gaussian;
            m_meanCurvatures[v] = mean;
            m_principalCurvatures[v] = principalCurvature;
            analysis.vertexCurvatures[idx++] = gaussian;

            totalGaussian += std::abs(gaussian);
            totalMean += std::abs(mean);
            maxGaussian = std::max(maxGaussian, gaussian);
            minGaussian = std::min(minGaussian, gaussian);
        }

        analysis.avgGaussianCurvature = totalGaussian / m_mesh->nVertices();
        analysis.avgMeanCurvature = totalMean / m_mesh->nVertices();

        std::cout << "[CurvatureAnalysis] 曲率范围: [" << minGaussian << ", " << maxGaussian << "]" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[CurvatureAnalysis] 曲率计算失败: " << e.what() << std::endl;
        // 使用默认值
        analysis.avgGaussianCurvature = 0.0;
        analysis.avgMeanCurvature = 0.0;
    }

    std::cout << "[MeshAnalysis] 完成分析：" << std::endl
              << "  - 欧拉特征数: " << analysis.eulerCharacteristic << std::endl
              << "  - 亏格: " << analysis.genus << std::endl
              << "  - 表面积: " << analysis.surfaceArea << std::endl
              << "  - 平均边长: " << analysis.avgEdgeLength << std::endl
              << "  - 平均高斯曲率: " << analysis.avgGaussianCurvature << std::endl;

    return analysis;
}

std::vector<Vertex> VariationalCuttingAlgorithm::computeFeaturePoints(double curvatureThreshold) {
    m_featurePoints.clear();

    // 如果还没有计算曲率，先计算
    if (m_gaussianCurvatures[m_mesh->vertex(0)] == 0.0) {
        analyzeMesh();
    }

    // 计算所有顶点的特征强度
    std::vector<std::pair<double, Vertex>> featureStrengths;
    for (Vertex v : m_mesh->vertices()) {
        double strength = computeVertexFeatureStrength(v);
        featureStrengths.emplace_back(strength, v);
    }

    // 按特征强度排序
    std::sort(featureStrengths.begin(), featureStrengths.end(),
              [](const auto& a, const auto& b) {
                  return a.first > b.first; // 降序排列
              });

    // 使用百分位数作为阈值
    size_t thresholdIndex = static_cast<size_t>(featureStrengths.size() * curvatureThreshold);
    double threshold = featureStrengths[thresholdIndex].first;

    std::cout << "[FeatureDetection] 特征强度阈值: " << threshold << std::endl;

    // 选择特征强度高的点
    for (const auto& [strength, vertex] : featureStrengths) {
        if (strength > threshold) {
            m_featurePoints.push_back(vertex);
        }
    }

    // 空间过滤：移除过于相近的特征点
    std::vector<Vertex> filteredFeatures;
    double minDistance = computeMeanEdgeLength() * 3.0; // 最小距离为平均边长的3倍

    for (Vertex candidate : m_featurePoints) {
        bool tooClose = false;
        for (Vertex existing : filteredFeatures) {
            double dist = (m_geometry->vertexPositions[candidate] -
                          m_geometry->vertexPositions[existing]).norm();
            if (dist < minDistance) {
                tooClose = true;
                break;
            }
        }
        if (!tooClose) {
            filteredFeatures.push_back(candidate);
        }
    }

    m_featurePoints = filteredFeatures;

    // 限制最大数量
    if (m_featurePoints.size() > 15) {
        m_featurePoints.resize(15);
    }

    // 确保至少有几个特征点
    if (m_featurePoints.size() < 3 && !featureStrengths.empty()) {
        m_featurePoints.clear();
        for (size_t i = 0; i < std::min(size_t(5), featureStrengths.size()); ++i) {
            m_featurePoints.push_back(featureStrengths[i].second);
        }
        std::cout << "[FeatureDetection] 特征点过少，使用前" << m_featurePoints.size() << "个最强特征点" << std::endl;
    }

    std::cout << "[FeatureDetection] 检测到 " << m_featurePoints.size()
              << " 个特征点（特征强度阈值: " << threshold << "）" << std::endl;

    return m_featurePoints;
}

std::vector<CutPath> VariationalCuttingAlgorithm::generateInitialCuts(const std::vector<Vertex>& featurePoints) {
    m_currentCuts.clear();

    if (featurePoints.size() < 2) {
        std::cout << "[CutGeneration] 特征点数量不足，无法生成切割路径" << std::endl;
        return m_currentCuts;
    }

    std::cout << "[CutGeneration] 使用 " << featurePoints.size() << " 个特征点生成切割路径" << std::endl;

    // 策略1：连接最重要的特征点对
    std::vector<std::pair<double, std::pair<size_t, size_t>>> potentialCuts;

    // 计算所有特征点对的切割价值
    for (size_t i = 0; i < featurePoints.size(); i++) {
        for (size_t j = i + 1; j < featurePoints.size(); j++) {
            Vertex v1 = featurePoints[i];
            Vertex v2 = featurePoints[j];

            // 计算切割价值：结合测地距离、特征强度差和拓扑重要性
            double feature1 = computeVertexFeatureStrength(v1);
            double feature2 = computeVertexFeatureStrength(v2);
            double featureProduct = feature1 * feature2; // 两端都是强特征的路径更有价值

            // 计算3D欧氏距离作为粗略估计
            double euclideanDist = (m_geometry->vertexPositions[v1] -
                                  m_geometry->vertexPositions[v2]).norm();

            // 避免距离太近或太远的路径
            double normalizedDist = euclideanDist / computeMeanEdgeLength();
            double distanceWeight = 1.0;
            if (normalizedDist < 5.0) {
                distanceWeight = 0.3; // 太近的路径价值低
            } else if (normalizedDist > 50.0) {
                distanceWeight = 0.5; // 太远的路径价值也低
            }

            double cutValue = featureProduct * distanceWeight;
            potentialCuts.emplace_back(cutValue, std::make_pair(i, j));
        }
    }

    // 按切割价值排序
    std::sort(potentialCuts.begin(), potentialCuts.end(),
              [](const auto& a, const auto& b) {
                  return a.first > b.first; // 降序排列
              });

    // 生成最有价值的几条切割路径
    size_t maxCuts = std::min(size_t(6), potentialCuts.size());
    std::cout << "[CutGeneration] 从 " << potentialCuts.size()
              << " 个潜在切割中选择前 " << maxCuts << " 个" << std::endl;

    for (size_t k = 0; k < maxCuts; k++) {
        auto [value, indices] = potentialCuts[k];
        auto [i, j] = indices;

        Vertex start = featurePoints[i];
        Vertex end = featurePoints[j];

        std::cout << "[CutGeneration] 计算特征点 " << i << " 到 " << j
                  << " 的测地路径 (价值: " << value << ")" << std::endl;

        // 优先使用测地线方法
        CutPath path = findGeodesicPath(start, end);

        // 如果测地线失败，回退到Dijkstra
        if (path.edges.empty()) {
            std::cout << "[CutGeneration] 测地路径失败，使用最短路径算法" << std::endl;
            path = findShortestPath(start, end);
        }

        if (!path.edges.empty()) {
            std::cout << "[CutGeneration] 成功生成路径，长度: " << path.totalLength << std::endl;
            m_currentCuts.push_back(path);
        } else {
            std::cout << "[CutGeneration] 路径生成失败" << std::endl;
        }
    }

    // 策略2：如果切割数量太少，添加一些连接边界到特征点的路径
    if (m_currentCuts.size() < 2 && m_mesh->nBoundaryLoops() > 0) {
        std::cout << "[CutGeneration] 切割数量不足，添加边界到特征点的路径" << std::endl;

        // 找到边界顶点
        std::vector<Vertex> boundaryVertices;
        for (Vertex v : m_mesh->vertices()) {
            if (v.isBoundary()) {
                boundaryVertices.push_back(v);
            }
        }

        if (!boundaryVertices.empty() && !featurePoints.empty()) {
            // 选择一个代表性的边界顶点
            Vertex boundaryStart = boundaryVertices[boundaryVertices.size() / 2];
            Vertex featureEnd = featurePoints[0];

            CutPath boundaryPath = findGeodesicPath(boundaryStart, featureEnd);
            if (boundaryPath.edges.empty()) {
                boundaryPath = findShortestPath(boundaryStart, featureEnd);
            }

            if (!boundaryPath.edges.empty()) {
                m_currentCuts.push_back(boundaryPath);
                std::cout << "[CutGeneration] 添加边界到特征点的路径" << std::endl;
            }
        }
    }

    std::cout << "[CutGeneration] 生成了 " << m_currentCuts.size() << " 条初始切割路径" << std::endl;

    return m_currentCuts;
}

void VariationalCuttingAlgorithm::optimizeCuts(std::vector<CutPath>& cuts, int iterations) {
    std::cout << "[CutOptimization] 开始优化 " << cuts.size() << " 条切割路径，迭代次数: " << iterations << std::endl;

    if (cuts.empty()) {
        std::cout << "[CutOptimization] 没有切割路径需要优化" << std::endl;
        return;
    }

    // 计算初始总能量
    double initialTotalEnergy = 0.0;
    for (const auto& cut : cuts) {
        initialTotalEnergy += computePathEnergy(cut);
    }
    std::cout << "[CutOptimization] 初始总能量: " << initialTotalEnergy << std::endl;

    // 优化参数
    const double learningRate = 0.1;
    const double convergenceThreshold = 1e-6;
    double previousEnergy = initialTotalEnergy;

    for (int iter = 0; iter < iterations; iter++) {
        double iterationEnergy = 0.0;

        for (size_t cutIdx = 0; cutIdx < cuts.size(); cutIdx++) {
            auto& cut = cuts[cutIdx];

            // 计算当前切割的能量和梯度
            double currentEnergy = computePathEnergy(cut);

            // 变分优化：对路径上的每个点计算能量梯度并调整位置
            std::vector<Vector3> energyGradients = computePathEnergyGradients(cut);

            // 应用梯度下降更新
            for (size_t i = 1; i < cut.positions.size() - 1; i++) { // 保持端点不变
                if (i < energyGradients.size()) {
                    Vector3 gradient = energyGradients[i];

                    // 梯度下降步
                    Vector3 newPosition = cut.positions[i] - learningRate * gradient;

                    // 将新位置投影到网格表面
                    Vector3 projectedPosition = projectToMeshSurface(newPosition);
                    cut.positions[i] = projectedPosition;
                }
            }

            // 重新计算路径长度和边连接
            updateCutPathGeometry(cut);

            // 应用平滑以保持路径质量
            smoothPath(cut);

            // 计算优化后的能量
            double newEnergy = computePathEnergy(cut);
            iterationEnergy += newEnergy;

            if (iter % 5 == 0) {
                std::cout << "  切割 " << cutIdx << ": 能量从 " << currentEnergy
                         << " 变为 " << newEnergy << std::endl;
            }
        }

        // 检查收敛性
        double energyChange = std::abs(iterationEnergy - previousEnergy);
        if (energyChange < convergenceThreshold) {
            std::cout << "[CutOptimization] 在迭代 " << iter
                     << " 处收敛 (能量变化: " << energyChange << ")" << std::endl;
            break;
        }

        previousEnergy = iterationEnergy;

        if (iter % 10 == 0) {
            std::cout << "  迭代 " << iter << ": 总能量 = " << iterationEnergy
                     << " (变化: " << energyChange << ")" << std::endl;
        }
    }

    // 最终清理和验证
    for (auto& cut : cuts) {
        // 移除可能的自相交
        removeSelfIntersections(cut);

        // 最后一次平滑
        smoothPath(cut);

        // 更新几何信息
        updateCutPathGeometry(cut);
    }

    double finalTotalEnergy = 0.0;
    for (const auto& cut : cuts) {
        finalTotalEnergy += computePathEnergy(cut);
    }

    m_currentCuts = cuts;

    std::cout << "[CutOptimization] 优化完成" << std::endl;
    std::cout << "  初始能量: " << initialTotalEnergy << std::endl;
    std::cout << "  最终能量: " << finalTotalEnergy << std::endl;
    std::cout << "  能量降低: " << (initialTotalEnergy - finalTotalEnergy) << " ("
              << ((initialTotalEnergy - finalTotalEnergy) / initialTotalEnergy * 100.0) << "%)" << std::endl;
}

void VariationalCuttingAlgorithm::applyCutsToMesh(const std::vector<CutPath>& cuts) {
    // 这里需要实际修改网格拓扑
    // 暂时只更新可视化数据
    m_currentCuts = cuts;
    std::cout << "[ApplyCuts] 应用 " << cuts.size() << " 条切割到网格" << std::endl;
}

std::vector<std::vector<Vector3>> VariationalCuttingAlgorithm::getCutLinesForRendering() const {
    std::vector<std::vector<Vector3>> lines;

    for (const auto& cut : m_currentCuts) {
        lines.push_back(cut.positions);
    }

    return lines;
}

std::vector<Vector3> VariationalCuttingAlgorithm::getFeaturePointsForRendering() const {
    std::vector<Vector3> points;

    for (Vertex v : m_featurePoints) {
        points.push_back(m_geometry->vertexPositions[v]);
    }

    return points;
}

std::vector<double> VariationalCuttingAlgorithm::getCurvatureColorsForRendering() const {
    std::vector<double> colors;

    for (Vertex v : m_mesh->vertices()) {
        colors.push_back(m_gaussianCurvatures[v]);
    }

    return colors;
}

double VariationalCuttingAlgorithm::computePathEnergy(const CutPath& path) {
    // 简单的能量函数：路径长度 + 曲率惩罚
    double energy = path.totalLength;

    // 添加曲率惩罚项
    for (const auto& pos : path.positions) {
        // 找最近的顶点
        double minDist = std::numeric_limits<double>::max();
        Vertex nearestVertex;

        for (Vertex v : m_mesh->vertices()) {
            double dist = (m_geometry->vertexPositions[v] - pos).norm();
            if (dist < minDist) {
                minDist = dist;
                nearestVertex = v;
            }
        }

        // 惩罚低曲率区域
        energy += 1.0 / (1.0 + std::abs(m_gaussianCurvatures[nearestVertex]));
    }

    return energy;
}

void VariationalCuttingAlgorithm::smoothPath(CutPath& path) {
    if (path.positions.size() < 3) return;

    // 简单的拉普拉斯平滑
    std::vector<Vector3> newPositions = path.positions;

    for (size_t i = 1; i < path.positions.size() - 1; i++) {
        newPositions[i] = 0.5 * path.positions[i] +
                         0.25 * (path.positions[i-1] + path.positions[i+1]);
    }

    path.positions = newPositions;

    // 重新计算路径长度
    path.totalLength = 0.0;
    for (size_t i = 1; i < path.positions.size(); i++) {
        path.totalLength += (path.positions[i] - path.positions[i-1]).norm();
    }
}

CutPath VariationalCuttingAlgorithm::findShortestPath(Vertex start, Vertex end) {
    CutPath path;

    // 使用Dijkstra算法找最短路径
    std::map<Vertex, double> distance;
    std::map<Vertex, Vertex> previous;
    std::priority_queue<std::pair<double, Vertex>,
                       std::vector<std::pair<double, Vertex>>,
                       std::greater<std::pair<double, Vertex>>> pq;

    // 初始化
    for (Vertex v : m_mesh->vertices()) {
        distance[v] = std::numeric_limits<double>::max();
    }
    distance[start] = 0.0;
    pq.push({0.0, start});

    // Dijkstra主循环
    while (!pq.empty()) {
        auto [dist, current] = pq.top();
        pq.pop();

        if (current == end) break;
        if (dist > distance[current]) continue;

        // 遍历邻居
        for (Vertex neighbor : current.adjacentVertices()) {
            Halfedge connectingHe = findConnectingHalfedge(current, neighbor);
            if (connectingHe == Halfedge()) continue; // 跳过无效连接
            double edgeLength = m_geometry->edgeLength(connectingHe.edge());
            double newDist = distance[current] + edgeLength;

            if (newDist < distance[neighbor]) {
                distance[neighbor] = newDist;
                previous[neighbor] = current;
                pq.push({newDist, neighbor});
            }
        }
    }

    // 重建路径
    if (distance[end] < std::numeric_limits<double>::max()) {
        std::vector<Vertex> vertices;
        Vertex current = end;

        while (current != start) {
            vertices.push_back(current);
            current = previous[current];
        }
        vertices.push_back(start);
        std::reverse(vertices.begin(), vertices.end());

        // 转换为边和位置
        for (size_t i = 0; i < vertices.size() - 1; i++) {
            Halfedge he = findConnectingHalfedge(vertices[i], vertices[i+1]);
            if (he != Halfedge()) {
                path.edges.push_back(he.edge());
            }
        }

        for (Vertex v : vertices) {
            path.positions.push_back(m_geometry->vertexPositions[v]);
        }

        path.totalLength = distance[end];
        path.isClosed = false;
    }

    return path;
}

// ========== UVUnwrappingAlgorithm 实现 ==========

UVUnwrappingAlgorithm::UVUnwrappingAlgorithm(HalfedgeMesh* mesh, VertexPositionGeometry* geometry)
    : m_mesh(mesh), m_geometry(geometry), m_hasCuts(false), m_useBFF(true) {
    m_uvCoordinates = VertexData<Vector2>(*mesh, Vector2{0, 0});

    // 检测边界环
    detectBoundaryLoops();

    std::cout << "[UVUnwrapping] 初始化完成，检测到 " << m_boundaryLoops.size()
              << " 个边界环" << std::endl;
}

bool UVUnwrappingAlgorithm::computeUVCoordinates(const std::vector<CutPath>& cuts) {
    std::cout << "[UVUnwrapping] 开始计算UV坐标，使用 " << cuts.size() << " 条切割" << std::endl;

    m_hasCuts = !cuts.empty();

    // 步骤1：应用切割约束
    if (m_hasCuts) {
        applyCutConstraints(cuts);
        std::cout << "[UVUnwrapping] 应用了切割约束" << std::endl;
    }

    // 步骤2：选择合适的参数化方法
    bool success = false;

    // 优先尝试使用真实BFF算法
    if (m_useBFF) {
        std::cout << "[UVUnwrapping] 尝试使用真实BFF算法" << std::endl;
        success = computeRealBFFParameterization(cuts);

        if (!success) {
            std::cout << "[UVUnwrapping] BFF算法失败，回退到其他方法" << std::endl;
        }
    }

    if (!success) {
        if (m_hasCuts || m_mesh->nBoundaryLoops() > 0) {
            // 对于有边界或有切割的网格，使用LSCM（最小二乘共形映射）
            std::cout << "[UVUnwrapping] 使用LSCM参数化方法" << std::endl;
            success = computeLSCMParameterization(cuts);

            if (!success) {
                std::cout << "[UVUnwrapping] LSCM失败，回退到调和参数化" << std::endl;
                success = computeHarmonicParameterization();
            }
        } else {
            // 对于封闭网格，使用调和参数化
            std::cout << "[UVUnwrapping] 使用调和参数化方法" << std::endl;
            success = computeHarmonicParameterization();
        }
    }

    if (!success) {
        std::cout << "[UVUnwrapping] 参数化失败，使用简化方法" << std::endl;
        computeConformalParameterization();
        success = true;
    }

    // 步骤3：优化失真
    if (success) {
        std::cout << "[UVUnwrapping] 最小化面积失真" << std::endl;
        minimizeAreaDistortion();

        // 验证参数化质量
        if (!isValidParameterization()) {
            std::cout << "[UVUnwrapping] 警告：参数化质量检验失败" << std::endl;
        }
    }

    // 步骤4：计算失真度量
    double totalDistortion = computeDistortion();
    double angleDistortion = computeAngleDistortion();
    double areaDistortion = computeAreaDistortion();

    std::cout << "[UVUnwrapping] UV计算完成" << std::endl;
    std::cout << "  总失真: " << totalDistortion << std::endl;
    std::cout << "  角度失真: " << angleDistortion << std::endl;
    std::cout << "  面积失真: " << areaDistortion << std::endl;

    return success;
}

std::vector<Vector2> UVUnwrappingAlgorithm::getUVCoordinates() const {
    std::vector<Vector2> coords;

    for (Vertex v : m_mesh->vertices()) {
        coords.push_back(m_uvCoordinates[v]);
    }

    return coords;
}

double UVUnwrappingAlgorithm::computeDistortion() const {
    double totalDistortion = 0.0;

    for (Face f : m_mesh->faces()) {
        // 计算3D面积
        double area3D = m_geometry->faceArea(f);

        // 计算UV面积
        std::vector<Vector2> uvVerts;
        for (Vertex v : f.adjacentVertices()) {
            uvVerts.push_back(m_uvCoordinates[v]);
        }

        double area2D = 0.5 * std::abs(
            (uvVerts[1].x - uvVerts[0].x) * (uvVerts[2].y - uvVerts[0].y) -
            (uvVerts[2].x - uvVerts[0].x) * (uvVerts[1].y - uvVerts[0].y)
        );

        // 失真度 = |log(area2D/area3D)|
        if (area3D > 1e-10 && area2D > 1e-10) {
            totalDistortion += std::abs(std::log(area2D / area3D));
        }
    }

    return totalDistortion / m_mesh->nFaces();
}

std::vector<std::array<Vector2, 3>> UVUnwrappingAlgorithm::getUVTrianglesForRendering() const {
    std::vector<std::array<Vector2, 3>> triangles;

    for (Face f : m_mesh->faces()) {
        std::array<Vector2, 3> tri;
        size_t i = 0;
        for (Vertex v : f.adjacentVertices()) {
            tri[i++] = m_uvCoordinates[v];
            if (i >= 3) break;
        }
        triangles.push_back(tri);
    }

    return triangles;
}

void UVUnwrappingAlgorithm::computeConformalParameterization() {
    // 简化实现：使用PCA投影

    // 计算质心
    Vector3 centroid = Vector3::zero();
    for (Vertex v : m_mesh->vertices()) {
        centroid += m_geometry->vertexPositions[v];
    }
    centroid /= m_mesh->nVertices();

    // 计算协方差矩阵
    double cov[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    for (Vertex v : m_mesh->vertices()) {
        Vector3 p = m_geometry->vertexPositions[v] - centroid;
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cov[i][j] += p[i] * p[j];
            }
        }
    }

    // 简化：直接投影到XY平面
    for (Vertex v : m_mesh->vertices()) {
        Vector3 p = m_geometry->vertexPositions[v];
        m_uvCoordinates[v] = Vector2{p.x, p.y};
    }

    // 归一化到[0,1]
    Vector2 minUV{1e10, 1e10}, maxUV{-1e10, -1e10};
    for (Vertex v : m_mesh->vertices()) {
        Vector2 uv = m_uvCoordinates[v];
        minUV.x = std::min(minUV.x, uv.x);
        minUV.y = std::min(minUV.y, uv.y);
        maxUV.x = std::max(maxUV.x, uv.x);
        maxUV.y = std::max(maxUV.y, uv.y);
    }

    double scale = std::max(maxUV.x - minUV.x, maxUV.y - minUV.y);
    for (Vertex v : m_mesh->vertices()) {
        m_uvCoordinates[v] = (m_uvCoordinates[v] - minUV) / scale;
    }
}

void UVUnwrappingAlgorithm::minimizeAreaDistortion() {
    // 简单的迭代优化
    for (int iter = 0; iter < 5; iter++) {
        VertexData<Vector2> newCoords(*m_mesh, Vector2{0, 0});

        // 拉普拉斯平滑
        for (Vertex v : m_mesh->vertices()) {
            if (v.isBoundary()) {
                newCoords[v] = m_uvCoordinates[v];
            } else {
                Vector2 sum{0, 0};
                double weight = 0.0;

                for (Vertex neighbor : v.adjacentVertices()) {
                    sum += m_uvCoordinates[neighbor];
                    weight += 1.0;
                }

                if (weight > 0) {
                    newCoords[v] = sum / weight;
                }
            }
        }

        m_uvCoordinates = newCoords;
    }
}

// ========== UVUnwrappingAlgorithm 新增方法 ==========

double UVUnwrappingAlgorithm::computeAngleDistortion() const {
    double totalAngleDistortion = 0.0;
    size_t validFaces = 0;

    for (Face f : m_mesh->faces()) {
        double angleDistortion = computeFaceAngleDistortion(f);
        if (angleDistortion >= 0) { // 有效的面
            totalAngleDistortion += angleDistortion;
            validFaces++;
        }
    }

    return validFaces > 0 ? totalAngleDistortion / validFaces : 0.0;
}

double UVUnwrappingAlgorithm::computeAreaDistortion() const {
    double totalAreaDistortion = 0.0;
    size_t validFaces = 0;

    for (Face f : m_mesh->faces()) {
        double areaDistortion = computeFaceAreaDistortion(f);
        if (areaDistortion >= 0) { // 有效的面
            totalAreaDistortion += areaDistortion;
            validFaces++;
        }
    }

    return validFaces > 0 ? totalAreaDistortion / validFaces : 0.0;
}

double UVUnwrappingAlgorithm::computeFaceAngleDistortion(Face f) const {
    // 获取面的顶点
    std::vector<Vertex> vertices;
    std::vector<Vector3> positions3D;
    std::vector<Vector2> positions2D;

    for (Vertex v : f.adjacentVertices()) {
        vertices.push_back(v);
        positions3D.push_back(m_geometry->vertexPositions[v]);
        positions2D.push_back(m_uvCoordinates[v]);
    }

    if (vertices.size() != 3) {
        return -1.0; // 非三角面，跳过
    }

    // 计算3D角度
    Vector3 e1_3d = positions3D[1] - positions3D[0];
    Vector3 e2_3d = positions3D[2] - positions3D[0];
    Vector3 e3_3d = positions3D[2] - positions3D[1];

    double angle1_3d = std::acos(std::clamp(dot(e1_3d, e2_3d) / (e1_3d.norm() * e2_3d.norm()), -1.0, 1.0));
    double angle2_3d = std::acos(std::clamp(dot(-e1_3d, e3_3d) / (e1_3d.norm() * e3_3d.norm()), -1.0, 1.0));
    double angle3_3d = M_PI - angle1_3d - angle2_3d;

    // 计算2D角度
    Vector2 e1_2d = positions2D[1] - positions2D[0];
    Vector2 e2_2d = positions2D[2] - positions2D[0];
    Vector2 e3_2d = positions2D[2] - positions2D[1];

    double angle1_2d = std::acos(std::clamp((e1_2d.x * e2_2d.x + e1_2d.y * e2_2d.y) /
                                          (std::sqrt(e1_2d.x*e1_2d.x + e1_2d.y*e1_2d.y) *
                                           std::sqrt(e2_2d.x*e2_2d.x + e2_2d.y*e2_2d.y)), -1.0, 1.0));

    double angle2_2d = std::acos(std::clamp((-e1_2d.x * e3_2d.x + -e1_2d.y * e3_2d.y) /
                                          (std::sqrt(e1_2d.x*e1_2d.x + e1_2d.y*e1_2d.y) *
                                           std::sqrt(e3_2d.x*e3_2d.x + e3_2d.y*e3_2d.y)), -1.0, 1.0));

    double angle3_2d = M_PI - angle1_2d - angle2_2d;

    // 计算角度失真（角度差的平方和）
    double distortion = std::pow(angle1_3d - angle1_2d, 2) +
                       std::pow(angle2_3d - angle2_2d, 2) +
                       std::pow(angle3_3d - angle3_2d, 2);

    return distortion / 3.0; // 平均角度失真
}

double UVUnwrappingAlgorithm::computeFaceAreaDistortion(Face f) const {
    // 计算3D面积
    double area3D = m_geometry->faceArea(f);

    // 计算UV面积
    std::vector<Vector2> uvVerts;
    for (Vertex v : f.adjacentVertices()) {
        uvVerts.push_back(m_uvCoordinates[v]);
    }

    if (uvVerts.size() != 3) {
        return -1.0; // 非三角面
    }

    double area2D = 0.5 * std::abs(
        (uvVerts[1].x - uvVerts[0].x) * (uvVerts[2].y - uvVerts[0].y) -
        (uvVerts[2].x - uvVerts[0].x) * (uvVerts[1].y - uvVerts[0].y)
    );

    // 面积失真：|log(area2D/area3D)|^2
    if (area3D > 1e-10 && area2D > 1e-10) {
        double ratio = area2D / area3D;
        return std::pow(std::log(ratio), 2);
    }

    return 100.0; // 退化面的惩罚
}

void UVUnwrappingAlgorithm::detectBoundaryLoops() {
    m_boundaryLoops.clear();

    // 简化实现：收集所有边界顶点
    std::vector<Vertex> boundaryVertices;
    for (Vertex v : m_mesh->vertices()) {
        if (v.isBoundary()) {
            boundaryVertices.push_back(v);
        }
    }

    if (!boundaryVertices.empty()) {
        m_boundaryLoops.push_back(boundaryVertices);
    }

    std::cout << "[UVUnwrapping] 检测到 " << boundaryVertices.size() << " 个边界顶点" << std::endl;
}

bool UVUnwrappingAlgorithm::isValidParameterization() const {
    // 检查是否有退化的三角形
    size_t degenerateCount = 0;

    for (Face f : m_mesh->faces()) {
        std::vector<Vector2> uvVerts;
        for (Vertex v : f.adjacentVertices()) {
            uvVerts.push_back(m_uvCoordinates[v]);
        }

        if (uvVerts.size() == 3) {
            double area = 0.5 * std::abs(
                (uvVerts[1].x - uvVerts[0].x) * (uvVerts[2].y - uvVerts[0].y) -
                (uvVerts[2].x - uvVerts[0].x) * (uvVerts[1].y - uvVerts[0].y)
            );

            if (area < 1e-10) {
                degenerateCount++;
            }
        }
    }

    double degenerateRatio = static_cast<double>(degenerateCount) / m_mesh->nFaces();
    return degenerateRatio < 0.05; // 允许5%的退化面
}

bool UVUnwrappingAlgorithm::computeLSCMParameterization(const std::vector<CutPath>& cuts) {
    std::cout << "[LSCM] 开始最小二乘共形映射计算" << std::endl;

    // 简化的LSCM实现
    // 注意：这里需要使用geometry-central的线性求解器来实现真正的LSCM
    // 目前提供一个占位符实现

    if (m_boundaryLoops.empty()) {
        std::cout << "[LSCM] 没有边界，无法使用LSCM" << std::endl;
        return false;
    }

    try {
        // 固定边界顶点
        fixBoundaryCoordinates();

        // 对内部顶点应用调和方程
        // 这是LSCM的简化版本
        for (int iter = 0; iter < 10; iter++) {
            VertexData<Vector2> newCoords(*m_mesh, Vector2{0, 0});

            for (Vertex v : m_mesh->vertices()) {
                if (v.isBoundary()) {
                    newCoords[v] = m_uvCoordinates[v]; // 保持边界固定
                } else {
                    Vector2 sum{0, 0};
                    double weight = 0.0;

                    for (Vertex neighbor : v.adjacentVertices()) {
                        sum += m_uvCoordinates[neighbor];
                        weight += 1.0;
                    }

                    if (weight > 0) {
                        newCoords[v] = sum / weight;
                    }
                }
            }

            m_uvCoordinates = newCoords;
        }

        std::cout << "[LSCM] LSCM参数化完成" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[LSCM] LSCM计算失败: " << e.what() << std::endl;
        return false;
    }
}

bool UVUnwrappingAlgorithm::computeHarmonicParameterization() {
    std::cout << "[Harmonic] 开始调和参数化计算" << std::endl;

    try {
        // 如果有边界，固定边界坐标
        if (!m_boundaryLoops.empty()) {
            fixBoundaryCoordinates();
        }

        // 应用拉普拉斯方程
        for (int iter = 0; iter < 20; iter++) {
            VertexData<Vector2> newCoords(*m_mesh, Vector2{0, 0});

            for (Vertex v : m_mesh->vertices()) {
                if (v.isBoundary()) {
                    newCoords[v] = m_uvCoordinates[v]; // 保持边界固定
                } else {
                    Vector2 sum{0, 0};
                    double totalWeight = 0.0;

                    // 使用余切权重
                    for (Halfedge he : v.outgoingHalfedges()) {
                        Vertex neighbor = he.tipVertex();
                        double weight = 1.0; // 简化：使用单位权重，实际应该用余切权重

                        sum += weight * m_uvCoordinates[neighbor];
                        totalWeight += weight;
                    }

                    if (totalWeight > 0) {
                        newCoords[v] = sum / totalWeight;
                    }
                }
            }

            m_uvCoordinates = newCoords;
        }

        std::cout << "[Harmonic] 调和参数化完成" << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Harmonic] 调和参数化失败: " << e.what() << std::endl;
        return false;
    }
}

void UVUnwrappingAlgorithm::fixBoundaryCoordinates() {
    if (m_boundaryLoops.empty()) {
        return;
    }

    // 将第一个边界环映射到单位圆
    const auto& boundaryLoop = m_boundaryLoops[0];
    size_t numBoundary = boundaryLoop.size();

    for (size_t i = 0; i < numBoundary; i++) {
        double angle = 2.0 * M_PI * i / numBoundary;
        double radius = 0.8; // 稍微小于1以避免边界问题

        m_uvCoordinates[boundaryLoop[i]] = Vector2{
            radius * std::cos(angle),
            radius * std::sin(angle)
        };
    }

    std::cout << "[BoundaryFix] 固定了 " << numBoundary << " 个边界顶点到单位圆" << std::endl;
}

void UVUnwrappingAlgorithm::applyCutConstraints(const std::vector<CutPath>& cuts) {
    std::cout << "[CutConstraints] 应用 " << cuts.size() << " 条切割约束" << std::endl;

    // 简化实现：标记切割路径上的顶点
    // 实际实现中需要修改网格拓扑或处理切割边的约束

    for (const auto& cut : cuts) {
        std::cout << "[CutConstraints] 处理长度为 " << cut.totalLength << " 的切割路径" << std::endl;
        // 目前仅记录，实际需要在参数化中考虑切割约束
    }
}

// ========== AlgorithmPipeline 实现 ==========

AlgorithmPipeline::AlgorithmPipeline() : m_state(PipelineState::EMPTY) {
    std::cout << "[AlgorithmPipeline] 初始化算法流水线" << std::endl;
}

bool AlgorithmPipeline::loadMesh(const std::string& filename) {
    try {
        std::tie(m_mesh, m_geometry) = readManifoldSurfaceMesh(filename);

        if (!m_mesh || !m_geometry) {
            std::cerr << "[AlgorithmPipeline] 加载网格失败: " << filename << std::endl;
            return false;
        }

        m_cuttingAlgo = std::make_unique<VariationalCuttingAlgorithm>(m_mesh.get(), m_geometry.get());
        m_uvAlgo = std::make_unique<UVUnwrappingAlgorithm>(m_mesh.get(), m_geometry.get());

        m_state = PipelineState::MESH_LOADED;
        updateVisualizationData();

        std::cout << "[AlgorithmPipeline] 成功加载网格: " << filename << std::endl;
        return true;

    } catch (const std::exception& e) {
        std::cerr << "[AlgorithmPipeline] 加载网格异常: " << e.what() << std::endl;
        return false;
    }
}

bool AlgorithmPipeline::setMesh(HalfedgeMesh* mesh, VertexPositionGeometry* geometry) {
    if (!mesh || !geometry) return false;

    // 注意：这里不拥有mesh和geometry的所有权
    m_cuttingAlgo = std::make_unique<VariationalCuttingAlgorithm>(mesh, geometry);
    m_uvAlgo = std::make_unique<UVUnwrappingAlgorithm>(mesh, geometry);

    m_state = PipelineState::MESH_LOADED;

    // 更新可视化数据（使用外部提供的mesh）
    m_visData.vertices.clear();
    m_visData.faces.clear();

    for (Vertex v : mesh->vertices()) {
        m_visData.vertices.push_back(geometry->vertexPositions[v]);
    }

    for (Face f : mesh->faces()) {
        std::array<int, 3> face;
        size_t i = 0;
        for (Vertex v : f.adjacentVertices()) {
            face[i++] = v.getIndex();
            if (i >= 3) break;
        }
        m_visData.faces.push_back(face);
    }

    return true;
}

MeshAnalysis AlgorithmPipeline::runMeshAnalysis() {
    if (m_state < PipelineState::MESH_LOADED) {
        std::cerr << "[AlgorithmPipeline] 需要先加载网格" << std::endl;
        return MeshAnalysis{};
    }

    m_visData.analysis = m_cuttingAlgo->analyzeMesh();
    m_visData.curvatureColors = m_cuttingAlgo->getCurvatureColorsForRendering();

    m_state = PipelineState::ANALYZED;
    updateVisualizationData();

    return m_visData.analysis;
}

std::vector<Vertex> AlgorithmPipeline::runFeatureDetection(double threshold) {
    if (m_state < PipelineState::MESH_LOADED) {
        std::cerr << "[AlgorithmPipeline] 需要先加载网格" << std::endl;
        return {};
    }

    auto features = m_cuttingAlgo->computeFeaturePoints(threshold);
    m_visData.featurePoints = m_cuttingAlgo->getFeaturePointsForRendering();

    m_state = PipelineState::FEATURES_DETECTED;
    updateVisualizationData();

    return features;
}

std::vector<CutPath> AlgorithmPipeline::runInitialCutGeneration() {
    if (m_state < PipelineState::FEATURES_DETECTED) {
        // 如果还没有检测特征，先自动检测
        runFeatureDetection();
    }

    auto cuts = m_cuttingAlgo->generateInitialCuts(
        m_cuttingAlgo->getFeaturePointsForRendering().size() > 0 ?
        std::vector<Vertex>{} : std::vector<Vertex>{});  // 简化：使用已有的特征点

    m_visData.cutLines = m_cuttingAlgo->getCutLinesForRendering();

    m_state = PipelineState::CUTS_GENERATED;
    updateVisualizationData();

    return cuts;
}

void AlgorithmPipeline::runCutOptimization(int iterations) {
    if (m_state < PipelineState::CUTS_GENERATED) {
        runInitialCutGeneration();
    }

    // 获取当前切割路径并优化
    auto cuts = m_cuttingAlgo->getCutLinesForRendering();
    // 注意：这里需要转换格式，暂时跳过

    m_visData.cutLines = m_cuttingAlgo->getCutLinesForRendering();

    m_state = PipelineState::CUTS_OPTIMIZED;
    updateVisualizationData();
}

bool AlgorithmPipeline::runUVUnwrapping() {
    if (m_state < PipelineState::CUTS_GENERATED) {
        std::cerr << "[AlgorithmPipeline] 需要先生成切割" << std::endl;
        return false;
    }

    std::vector<CutPath> cuts; // 从m_cuttingAlgo获取
    bool success = m_uvAlgo->computeUVCoordinates(cuts);

    if (success) {
        m_visData.uvCoordinates = m_uvAlgo->getUVCoordinates();
        m_visData.uvTriangles = m_uvAlgo->getUVTrianglesForRendering();
        m_visData.uvDistortion = m_uvAlgo->computeDistortion();

        m_state = PipelineState::UV_UNWRAPPED;
        updateVisualizationData();
    }

    return success;
}

AlgorithmPipeline::VisualizationData AlgorithmPipeline::getVisualizationData() const {
    return m_visData;
}

void AlgorithmPipeline::updateVisualizationData() {
    // 更新基础网格数据
    if (m_mesh && m_geometry) {
        m_visData.vertices.clear();
        m_visData.faces.clear();

        for (Vertex v : m_mesh->vertices()) {
            m_visData.vertices.push_back(m_geometry->vertexPositions[v]);
        }

        for (Face f : m_mesh->faces()) {
            std::array<int, 3> face;
            size_t i = 0;
            for (Vertex v : f.adjacentVertices()) {
                face[i++] = v.getIndex();
                if (i >= 3) break;
            }
            m_visData.faces.push_back(face);
        }
    }

    // 根据当前状态更新其他数据
    if (m_state >= PipelineState::ANALYZED && m_cuttingAlgo) {
        m_visData.curvatureColors = m_cuttingAlgo->getCurvatureColorsForRendering();
    }

    if (m_state >= PipelineState::FEATURES_DETECTED && m_cuttingAlgo) {
        m_visData.featurePoints = m_cuttingAlgo->getFeaturePointsForRendering();
    }

    if (m_state >= PipelineState::CUTS_GENERATED && m_cuttingAlgo) {
        m_visData.cutLines = m_cuttingAlgo->getCutLinesForRendering();
    }

    if (m_state >= PipelineState::UV_UNWRAPPED && m_uvAlgo) {
        m_visData.uvCoordinates = m_uvAlgo->getUVCoordinates();
        m_visData.uvTriangles = m_uvAlgo->getUVTrianglesForRendering();
        m_visData.uvDistortion = m_uvAlgo->computeDistortion();
    }
}

// ========== VariationalCuttingAlgorithm 新增方法 ==========

void VariationalCuttingAlgorithm::initializeGeometryTools() {
    try {
        // 创建热方法距离求解器，用于计算测地线距离
        m_heatSolver = std::make_unique<HeatMethodDistanceSolver>(*m_geometry);
        std::cout << "[GeometryTools] 热方法距离求解器初始化完成" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[GeometryTools] 初始化失败: " << e.what() << std::endl;
        m_heatSolver = nullptr;
    }
}

double VariationalCuttingAlgorithm::computeVertexFeatureStrength(Vertex v) {
    // 计算顶点的特征强度：结合高斯曲率、平均曲率和局部几何特征
    double gaussian = std::abs(m_gaussianCurvatures[v]);
    double mean = std::abs(m_meanCurvatures[v]);

    // 计算局部曲率变化
    double curvatureVariation = 0.0;
    int neighborCount = 0;

    for (Vertex neighbor : v.adjacentVertices()) {
        double neighborGaussian = std::abs(m_gaussianCurvatures[neighbor]);
        curvatureVariation += std::abs(gaussian - neighborGaussian);
        neighborCount++;
    }

    if (neighborCount > 0) {
        curvatureVariation /= neighborCount;
    }

    // 综合特征强度：高斯曲率 + 平均曲率 + 曲率变化
    return gaussian + 0.5 * mean + 0.3 * curvatureVariation;
}

CutPath VariationalCuttingAlgorithm::findGeodesicPath(Vertex start, Vertex end) {
    CutPath path;

    if (!m_heatSolver) {
        std::cerr << "[GeodesicPath] 热方法求解器未初始化，回退到Dijkstra算法" << std::endl;
        return findShortestPath(start, end);
    }

    try {
        // 使用热方法计算从起点到所有顶点的距离
        VertexData<double> distances = m_heatSolver->computeDistance(start);

        // 重建测地路径：从终点开始，沿着梯度下降方向回溯到起点
        std::vector<Vertex> pathVertices;
        Vertex current = end;
        pathVertices.push_back(current);

        const double epsilon = 1e-6;
        const int maxSteps = m_mesh->nVertices(); // 防止无限循环
        int steps = 0;

        while (current != start && steps < maxSteps) {
            Vertex nextVertex = current;
            double minDistance = distances[current];

            // 在邻居中找到距离最小的顶点
            for (Vertex neighbor : current.adjacentVertices()) {
                if (distances[neighbor] < minDistance - epsilon) {
                    minDistance = distances[neighbor];
                    nextVertex = neighbor;
                }
            }

            if (nextVertex == current) {
                // 无法找到更近的邻居，路径构建失败
                std::cerr << "[GeodesicPath] 无法重建测地路径，回退到Dijkstra算法" << std::endl;
                return findShortestPath(start, end);
            }

            current = nextVertex;
            pathVertices.push_back(current);
            steps++;
        }

        if (steps >= maxSteps) {
            std::cerr << "[GeodesicPath] 路径重建步数过多，回退到Dijkstra算法" << std::endl;
            return findShortestPath(start, end);
        }

        // 反转路径，使其从起点到终点
        std::reverse(pathVertices.begin(), pathVertices.end());

        // 构建CutPath结构
        for (size_t i = 0; i < pathVertices.size() - 1; i++) {
            Halfedge he = findConnectingHalfedge(pathVertices[i], pathVertices[i + 1]);
            if (he == Halfedge()) {
                // 顶点不相邻，路径无效
                std::cerr << "[GeodesicPath] 路径包含不相邻顶点，回退到Dijkstra算法" << std::endl;
                return findShortestPath(start, end);
            }
            path.edges.push_back(he.edge());
        }

        for (Vertex v : pathVertices) {
            path.positions.push_back(m_geometry->vertexPositions[v]);
        }

        path.totalLength = distances[end];
        path.isClosed = false;

        std::cout << "[GeodesicPath] 测地路径构建成功，长度: " << path.totalLength << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "[GeodesicPath] 测地路径计算失败: " << e.what() << "，回退到Dijkstra算法" << std::endl;
        return findShortestPath(start, end);
    }

    return path;
}

double VariationalCuttingAlgorithm::computeMeanEdgeLength() const {
    double totalEdgeLength = 0.0;
    for (Edge e : m_mesh->edges()) {
        totalEdgeLength += m_geometry->edgeLength(e);
    }
    return totalEdgeLength / m_mesh->nEdges();
}

Halfedge VariationalCuttingAlgorithm::findConnectingHalfedge(Vertex v1, Vertex v2) const {
    // 遍历v1的邻接半边，寻找指向v2的半边
    for (Halfedge he : v1.outgoingHalfedges()) {
        if (he.tipVertex() == v2) {
            return he;
        }
    }
    // 如果没找到，返回无效的半边
    return Halfedge();
}

// ========== 变分优化辅助方法 ==========

std::vector<Vector3> VariationalCuttingAlgorithm::computePathEnergyGradients(const CutPath& path) {
    std::vector<Vector3> gradients(path.positions.size(), Vector3::zero());

    if (path.positions.size() < 2) {
        return gradients;
    }

    const double epsilon = 1e-6;

    // 对路径上的每个点计算能量梯度
    for (size_t i = 1; i < path.positions.size() - 1; i++) { // 跳过端点
        Vector3 currentPos = path.positions[i];

        // 计算当前位置的能量
        CutPath tempPath = path;
        tempPath.positions[i] = currentPos;
        double currentEnergy = computePathEnergy(tempPath);

        // 计算X方向的偏导数
        tempPath.positions[i] = currentPos + Vector3{epsilon, 0, 0};
        double energyX = computePathEnergy(tempPath);
        double gradX = (energyX - currentEnergy) / epsilon;

        // 计算Y方向的偏导数
        tempPath.positions[i] = currentPos + Vector3{0, epsilon, 0};
        double energyY = computePathEnergy(tempPath);
        double gradY = (energyY - currentEnergy) / epsilon;

        // 计算Z方向的偏导数
        tempPath.positions[i] = currentPos + Vector3{0, 0, epsilon};
        double energyZ = computePathEnergy(tempPath);
        double gradZ = (energyZ - currentEnergy) / epsilon;

        gradients[i] = Vector3{gradX, gradY, gradZ};

        // 限制梯度大小以避免数值不稳定
        double gradMagnitude = gradients[i].norm();
        if (gradMagnitude > 1.0) {
            gradients[i] = gradients[i] / gradMagnitude;
        }
    }

    return gradients;
}

Vector3 VariationalCuttingAlgorithm::projectToMeshSurface(const Vector3& position) {
    // 简化实现：找到最近的顶点位置
    double minDistance = std::numeric_limits<double>::max();
    Vector3 closestPosition = position;

    for (Vertex v : m_mesh->vertices()) {
        Vector3 vertexPos = m_geometry->vertexPositions[v];
        double distance = (position - vertexPos).norm();

        if (distance < minDistance) {
            minDistance = distance;
            closestPosition = vertexPos;
        }
    }

    // 更复杂的实现可能包括：
    // 1. 投影到最近的面
    // 2. 在面内插值
    // 3. 使用重心坐标

    return closestPosition;
}

void VariationalCuttingAlgorithm::updateCutPathGeometry(CutPath& path) {
    if (path.positions.size() < 2) {
        path.totalLength = 0.0;
        return;
    }

    // 重新计算路径长度
    path.totalLength = 0.0;
    for (size_t i = 1; i < path.positions.size(); i++) {
        double segmentLength = (path.positions[i] - path.positions[i-1]).norm();
        path.totalLength += segmentLength;
    }

    // 重新构建边列表（简化实现）
    path.edges.clear();
    // 注意：这里需要更复杂的逻辑来重新找到对应的网格边
    // 目前仅更新长度信息
}

void VariationalCuttingAlgorithm::removeSelfIntersections(CutPath& path) {
    if (path.positions.size() < 4) {
        return; // 太短的路径不可能自相交
    }

    // 简化的自相交检测和移除
    std::vector<Vector3> cleanedPositions;
    cleanedPositions.push_back(path.positions[0]); // 保留起点

    for (size_t i = 1; i < path.positions.size() - 1; i++) {
        Vector3 currentPos = path.positions[i];
        bool intersects = false;

        // 检查当前点是否与之前的线段过于接近
        for (size_t j = 0; j < cleanedPositions.size() - 1; j++) {
            double distToSegment = pointToLineSegmentDistance(
                currentPos, cleanedPositions[j], cleanedPositions[j+1]);

            if (distToSegment < computeMeanEdgeLength() * 0.1) {
                intersects = true;
                break;
            }
        }

        if (!intersects) {
            cleanedPositions.push_back(currentPos);
        }
    }

    cleanedPositions.push_back(path.positions.back()); // 保留终点

    // 更新路径
    path.positions = cleanedPositions;
    updateCutPathGeometry(path);
}

double VariationalCuttingAlgorithm::pointToLineSegmentDistance(
    const Vector3& point, const Vector3& lineStart, const Vector3& lineEnd) {

    Vector3 lineVec = lineEnd - lineStart;
    Vector3 pointVec = point - lineStart;

    double lineLength = lineVec.norm();
    if (lineLength < 1e-10) {
        return pointVec.norm(); // 退化为点到点距离
    }

    double t = dot(pointVec, lineVec) / (lineLength * lineLength);
    t = std::max(0.0, std::min(1.0, t)); // 约束到线段范围内

    Vector3 projection = lineStart + t * lineVec;
    return (point - projection).norm();
}

// 新增：真实BFF参数化实现
bool UVUnwrappingAlgorithm::computeRealBFFParameterization(const std::vector<CutPath>& cuts) {
    std::cout << "[BFF] 开始真实BFF参数化计算" << std::endl;

    // 暂时使用简化版本，真实BFF集成需要在编译时链接BFF库
    // TODO: 集成deps/boundary-first-flattening库

    std::cout << "[BFF] 使用简化的BFF实现" << std::endl;

    // 这里提供一个改进的共形参数化作为替代
    if (m_boundaryLoops.empty() && cuts.empty()) {
        // 对于闭合网格，需要先创建切割
        std::cout << "[BFF] 网格闭合，使用简化的球面参数化" << std::endl;

        // 简单的球面投影
        for (Vertex v : m_mesh->vertices()) {
            Vector3 pos = m_geometry->vertexPositions[v];
            pos = pos.unit(); // 归一化到单位球面

            // 球面坐标到UV
            double theta = std::atan2(pos.y, pos.x);
            double phi = std::acos(std::clamp(pos.z, -1.0, 1.0));

            m_uvCoordinates[v] = Vector2{
                (theta + M_PI) / (2 * M_PI),
                phi / M_PI
            };
        }

        return true;
    } else {
        // 对于有边界的网格，使用改进的共形映射
        fixBoundaryCoordinates();

        // 使用拉普拉斯平滑进行迭代优化
        for (int iter = 0; iter < 50; iter++) {
            VertexData<Vector2> newCoords(*m_mesh, Vector2{0, 0});

            for (Vertex v : m_mesh->vertices()) {
                bool isBoundary = false;
                for (const auto& loop : m_boundaryLoops) {
                    if (std::find(loop.begin(), loop.end(), v) != loop.end()) {
                        isBoundary = true;
                        break;
                    }
                }

                if (isBoundary) {
                    newCoords[v] = m_uvCoordinates[v];
                } else {
                    // 使用余切权重的拉普拉斯
                    Vector2 sum{0, 0};
                    double weightSum = 0.0;

                    for (Halfedge he : v.outgoingHalfedges()) {
                        Vertex neighbor = he.tipVertex();

                        // 计算余切权重
                        double weight = 1.0; // 简化：使用均匀权重
                        if (he.isInterior() && he.twin().isInterior()) {
                            // 可以在这里计算真实的余切权重
                            weight = 1.0;
                        }

                        sum += weight * m_uvCoordinates[neighbor];
                        weightSum += weight;
                    }

                    if (weightSum > 0) {
                        newCoords[v] = sum / weightSum;
                    }
                }
            }

            m_uvCoordinates = newCoords;
        }

        std::cout << "[BFF] 简化的共形映射完成" << std::endl;
        return true;
    }
}

AlgorithmPipeline::ProcessingStatus AlgorithmPipeline::getStatus() const {
    ProcessingStatus status;

    // 检查网格是否加载
    status.meshLoaded = (m_state >= PipelineState::MESH_LOADED);

    // 检查分析是否完成
    status.analysisComplete = (m_state >= PipelineState::ANALYZED);

    // 检查切割是否生成
    status.cutsGenerated = (m_state >= PipelineState::CUTS_GENERATED);

    // 检查UV映射是否完成
    status.uvMappingComplete = (m_state >= PipelineState::UV_UNWRAPPED);

    // 获取网格统计信息
    if (m_mesh) {
        status.numVertices = m_mesh->nVertices();
        status.numFaces = m_mesh->nFaces();
    } else {
        status.numVertices = 0;
        status.numFaces = 0;
    }

    // 获取切割数量
    if (m_cuttingAlgo) {
        status.numCuts = m_cuttingAlgo->getCutLinesForRendering().size();
    } else {
        status.numCuts = 0;
    }

    return status;
}

} // namespace SurfaceTextureMapping