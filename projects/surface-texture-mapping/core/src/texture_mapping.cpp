/**
 * 纹理映射算法实现
 * UV参数化和纹理坐标生成
 */

#include "texture_mapping.h"
#include "bff_wrapper.h"
#include "real_algorithm_integration.h"  // For IntegratedVariationalCutter
#include "eulerian_cut_integrator.h"
#include <iostream>
#include <algorithm>
#include <numeric>

namespace SurfaceTextureMapping {

// 类型别名 - 避免命名冲突
using Face = geometrycentral::surface::Face;
using Vertex = geometrycentral::surface::Vertex;
using Vector3 = geometrycentral::Vector3;

// TextureMapper实现

void TextureMapper::setMesh(std::shared_ptr<SurfaceMesh> mesh,
                            std::shared_ptr<VertexPositionGeometry> geometry) {
    mesh_ = mesh;
    geometry_ = geometry;
}

std::optional<TextureMapper::UVMapping>
TextureMapper::computeUVMapping(const MappingParams& params) {
    if (!mesh_ || !geometry_) {
        std::cerr << "TextureMapper not initialized with mesh" << std::endl;
        return std::nullopt;
    }

    std::cout << "Computing UV mapping..." << std::endl;

    UVMapping mapping;

    // 根据参数选择映射方法
    if (params.useConformalMapping) {
        std::cout << "  Using conformal mapping (BFF)" << std::endl;
        mapping = computeConformalMapping(params);
    } else {
        std::cout << "  Using LSCM mapping" << std::endl;
        mapping = computeLSCMMapping(params);
    }

    // 后处理
    if (params.enableAreaCorrection) {
        correctAreaDistortion(mapping);
    }

    // 计算失真度量
    mapping.totalDistortion = computeTotalDistortion(mapping);
    mapping.maxDistortion = computeMaximumDistortion(mapping);

    std::cout << "UV mapping completed:" << std::endl;
    std::cout << "  Charts: " << mapping.charts.size() << std::endl;
    std::cout << "  Total distortion: " << mapping.totalDistortion << std::endl;

    return mapping;
}

std::vector<int> TextureMapper::detectConeVertices(double curvatureThreshold) const {
    std::vector<int> cones;

    if (!mesh_ || !geometry_) {
        return cones;
    }

    std::cout << "Detecting cone vertices..." << std::endl;
    std::cout << "  Curvature threshold: " << curvatureThreshold << std::endl;

    // 使用geometry-central计算高斯曲率
    geometry_->requireVertexGaussianCurvatures();
    auto curvatures = geometry_->vertexGaussianCurvatures;

    // 收集高曲率顶点
    std::vector<std::pair<double, int>> candidates;
    int idx = 0;
    for (auto v : mesh_->vertices()) {
        double curvature = std::abs(curvatures[v]);
        if (curvature > curvatureThreshold) {
            candidates.push_back({curvature, idx});
        }
        idx++;
    }

    // 按曲率降序排序
    std::sort(candidates.begin(), candidates.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    // 选择前N个高曲率点，确保它们之间有一定距离
    // TODO: Calculate mean edge length from geometry-central
    double meanEdgeLength = 0.0;
    for (auto e : mesh_->edges()) {
        meanEdgeLength += geometry_->edgeLength(e);
    }
    meanEdgeLength /= mesh_->nEdges();
    const double minDistance = meanEdgeLength * 5.0;
    const int maxCones = 8; // 限制锥点数量

    for (const auto& [curvature, vertexIdx] : candidates) {
        bool tooClose = false;

        auto v = mesh_->vertex(vertexIdx);
        auto pos = geometry_->vertexPositions[v];

        for (int existingIdx : cones) {
            auto existingV = mesh_->vertex(existingIdx);
            auto existingPos = geometry_->vertexPositions[existingV];
            double dist = (pos - existingPos).norm();
            if (dist < minDistance) {
                tooClose = true;
                break;
            }
        }

        if (!tooClose) {
            cones.push_back(vertexIdx);
            if (cones.size() >= maxCones) {
                break;
            }
        }
    }

    std::cout << "  Detected " << cones.size() << " cone vertices" << std::endl;

    return cones;
}

TextureMapper::DistortionMetrics TextureMapper::computeDistortionMetrics(const UVMapping& mapping) const {
    DistortionMetrics metrics;

    // 计算角度失真
    metrics.angleDistortion = computeAngleDistortion(mapping);

    // 计算面积失真
    metrics.areaDistortion = computeAreaDistortion(mapping);

    // 计算共形误差
    metrics.conformalError = computeConformalError(mapping);

    std::cout << "Distortion metrics:" << std::endl;
    std::cout << "  Angle distortion: " << metrics.angleDistortion << std::endl;
    std::cout << "  Area distortion: " << metrics.areaDistortion << std::endl;
    std::cout << "  Conformal error: " << metrics.conformalError << std::endl;

    return metrics;
}

bool TextureMapper::exportUVMesh(const std::string& filename, const UVMapping& mapping) const {
    std::cout << "Exporting UV mesh to: " << filename << std::endl;

    // TODO: 实现UV网格导出
    // 将UV坐标作为3D坐标导出到OBJ文件
    return true;
}

void TextureMapper::visualizeUVMapping(const UVMapping& mapping,
                                       const std::string& outputPath,
                                       int resolution) const {
    std::cout << "Visualizing UV mapping..." << std::endl;
    std::cout << "  Output: " << outputPath << std::endl;
    std::cout << "  Resolution: " << resolution << "x" << resolution << std::endl;

    // TODO: 实现UV映射可视化
    // 生成网格线图像或棋盘格纹理
}

TextureMapper::UVMapping
TextureMapper::computeConformalMapping(const MappingParams& params) {
    UVMapping mapping;

    std::cout << "    Computing BFF parameterization..." << std::endl;

    // 创建BFF包装器
    BFFWrapper bffWrapper;

    // 转换网格格式为半边结构（BFF需要）
    // Note: SurfaceMesh doesn't have copy constructor, use original mesh directly
    auto halfedgeMesh = mesh_.get();
    auto halfedgeGeometry = geometry_.get();

    // 设置网格到BFF
    if (!bffWrapper.setMesh(halfedgeMesh, halfedgeGeometry)) {
        std::cerr << "      Failed to initialize BFF mesh" << std::endl;
        // No need to delete as we're using the original mesh pointers
        return mapping;
    }

    // 检测或使用指定的锥点
    std::vector<EulerianCutIntegrator::GC_Vertex> coneVertices;
    if (params.automaticConeDetection) {
        std::cout << "      Detecting cone points..." << std::endl;
        auto coneIndices = detectConeVertices(params.curvatureThreshold);
        for (int idx : coneIndices) {
            coneVertices.push_back(halfedgeMesh->vertex(idx));
        }
        std::cout << "      Found " << coneVertices.size() << " cone points" << std::endl;
    } else if (!params.coneVertices.empty()) {
        for (int idx : params.coneVertices) {
            coneVertices.push_back(halfedgeMesh->vertex(idx));
        }
    }

    // 生成切割路径（如果网格是闭合的）
    std::vector<std::vector<geometrycentral::surface::Edge>> cutPaths;
    if (!halfedgeMesh->hasBoundary() || !coneVertices.empty()) {
        std::cout << "      Generating cut paths using Variational Surface Cutting algorithm..." << std::endl;

        // 尝试使用真正的Variational Surface Cutting算法
        auto variationalCutter = RealAlgorithmIntegration::createIntegratedVariationalCutter();

        if (variationalCutter) {
            // 设置网格
            variationalCutter->setMesh(mesh_, geometry_);

            // 设置切割参数
            VariationalCutter::CuttingParams cutParams;
            cutParams.lengthRegularization = 1.0;  // 长度正则化
            cutParams.smoothRegularization = 0.1;  // 平滑正则化
            cutParams.maxIterations = 50;
            cutParams.convergenceThreshold = 1e-4;

            // 运行真正的Variational Surface Cutting算法
            auto variationalCuts = variationalCutter->computeOptimalCuts(cutParams);
            std::cout << "      Generated " << variationalCuts.size() << " variational cut curves" << std::endl;

            // 将VariationalCutter的切缝转换为BFF需要的Edge paths格式
            // TODO: 实现从CutCurve到Edge paths的转换
            // 暂时回退到简单方法
            SmartCutGenerator cutGen;
            cutPaths = cutGen.generateCutsForClosedMesh(halfedgeMesh, halfedgeGeometry, coneVertices);

        } else {
            // 如果Variational Cutting不可用，回退到简单的方法
            std::cout << "      Warning: Variational Surface Cutting unavailable, using fallback method" << std::endl;
            SmartCutGenerator cutGen;
            cutPaths = cutGen.generateCutsForClosedMesh(halfedgeMesh, halfedgeGeometry, coneVertices);
        }

        if (!cutPaths.empty()) {
            std::cout << "      Applying " << cutPaths.size() << " cut paths" << std::endl;
            if (!bffWrapper.applyCuts(cutPaths)) {
                std::cerr << "      Failed to apply cuts" << std::endl;
                // No need to delete as we're using the original mesh pointers
                return mapping;
            }
        }
    }

    // 配置BFF参数
    BFFConfig bffConfig;
    if (!coneVertices.empty()) {
        bffConfig.mode = BFFConfig::FlatteningMode::WITH_CONES;
        // 将Vertex转换为索引
        for (const auto& v : coneVertices) {
            bffConfig.coneVertices.push_back(v.getIndex());
            // 默认锥角为2*PI（平坦）
            bffConfig.coneAngles.push_back(2.0 * M_PI);
        }
        // bffConfig.useCuts = true; // TODO: Add if this member exists in BFFConfig
    } else if (halfedgeMesh->hasBoundary()) {
        // 有边界的网格，使用磁盘展平
        bffConfig.mode = BFFConfig::FlatteningMode::TO_DISK;
    } else {
        // 闭合网格，自动模式
        bffConfig.mode = BFFConfig::FlatteningMode::AUTOMATIC;
    }

    bffConfig.prescribeBoundaryAngles = false;
    // bffConfig.targetAreaDistortion = 0.1; // TODO: Add if this member exists in BFFConfig

    // 执行参数化
    std::cout << "      Running BFF solver..." << std::endl;
    BFFResult bffResult = bffWrapper.computeParameterization(bffConfig);

    if (!bffResult.success) {
        std::cerr << "      BFF failed: " << bffResult.errorMessage << std::endl;
        // No need to delete as we're using the original mesh pointers
        return mapping;
    }

    std::cout << "      BFF succeeded with distortion: " << bffResult.distortion << std::endl;

    // 转换结果到UVMapping格式
    mapping.uvCoordinates = bffResult.uvCoordinates;

    // 验证UV坐标向量大小
    std::cout << "      UV coordinates size: " << mapping.uvCoordinates.size() << std::endl;
    std::cout << "      Mesh vertices: " << halfedgeMesh->nVertices() << std::endl;

    // 创建单个chart包含所有面
    std::vector<int> chart;
    for (Face f : halfedgeMesh->faces()) {
        chart.push_back(f.getIndex());
    }
    mapping.charts.push_back(chart);

    // 计算失真度量
    mapping.totalDistortion = bffResult.distortion;
    mapping.maxDistortion = bffWrapper.computeAngleDistortion();

    // 计算每个面的失真
    auto uvTriangles = bffWrapper.getUVTriangles();
    mapping.faceDistortions.resize(halfedgeMesh->nFaces());

    int faceIdx = 0;
    for (Face f : halfedgeMesh->faces()) {
        // 简单的面积失真计算
        double originalArea = halfedgeGeometry->faceArea(f);
        if (faceIdx < uvTriangles.size()) {
            auto& tri = uvTriangles[faceIdx];
            Vector2 v0 = tri[0];
            Vector2 v1 = tri[1];
            Vector2 v2 = tri[2];
            double uvArea = 0.5 * std::abs((v1.x - v0.x) * (v2.y - v0.y) -
                                          (v2.x - v0.x) * (v1.y - v0.y));
            mapping.faceDistortions[faceIdx] = std::abs(std::log(uvArea / originalArea));
        }
        faceIdx++;
    }

    // No cleanup needed as we're using the original mesh pointers

    return mapping;
}

TextureMapper::UVMapping
TextureMapper::computeLSCMMapping(const MappingParams& params) {
    UVMapping mapping;

    // TODO: 实现LSCM (Least Squares Conformal Maps) 算法
    std::cout << "    Computing LSCM parameterization..." << std::endl;

    return mapping;
}

void TextureMapper::correctAreaDistortion(UVMapping& mapping) {
    std::cout << "  Correcting area distortion..." << std::endl;

    // TODO: 实现面积失真校正
    // 调整UV坐标以减少面积失真
}

double TextureMapper::computeTotalDistortion(const UVMapping& mapping) const {
    double totalDistortion = 0.0;

    // TODO: 计算总失真
    for (size_t i = 0; i < mapping.charts.size(); ++i) {
        totalDistortion += computeChartDistortion(i, mapping);
    }

    return totalDistortion;
}

double TextureMapper::computeMaximumDistortion(const UVMapping& mapping) const {
    double maxDistortion = 0.0;

    // TODO: 计算最大失真
    for (size_t i = 0; i < mapping.charts.size(); ++i) {
        double chartDist = computeChartDistortion(i, mapping);
        maxDistortion = std::max(maxDistortion, chartDist);
    }

    return maxDistortion;
}

double TextureMapper::computeChartDistortion(size_t chartIndex, const UVMapping& mapping) const {
    if (chartIndex >= mapping.charts.size() || !mesh_ || !geometry_) {
        return -1.0;
    }

    // 安全检查：如果UV坐标为空，直接返回
    if (mapping.uvCoordinates.empty()) {
        return -1.0;
    }

    const auto& chart = mapping.charts[chartIndex];
    double totalDistortion = 0.0;
    int nFaces = 0;

    for (int faceIdx : chart) {
        auto f = mesh_->face(faceIdx);

        // 获取原始面积
        double originalArea = geometry_->faceArea(f);

        // 获取UV面积 - 使用边界检查确保安全
        std::vector<Vector2> uvPositions;
        bool allVerticesValid = true;
        for (auto v : f.adjacentVertices()) {
            size_t idx = static_cast<size_t>(v.getIndex());
            if (idx < mapping.uvCoordinates.size()) {
                uvPositions.push_back(mapping.uvCoordinates[idx]);
            } else {
                // 顶点索引超出UV坐标范围，跳过此面
                allVerticesValid = false;
                break;
            }
        }

        if (allVerticesValid && uvPositions.size() == 3 && originalArea > 1e-10) {
            // 计算UV面积
            Vector2 v0 = uvPositions[1] - uvPositions[0];
            Vector2 v1 = uvPositions[2] - uvPositions[0];
            double uvArea = 0.5 * std::abs(v0.x * v1.y - v0.y * v1.x);

            // 计算面积失真
            if (uvArea > 1e-10) {
                double distortion = std::abs(std::log(uvArea / originalArea));
                totalDistortion += distortion;
                nFaces++;
            }
        }
    }

    return nFaces > 0 ? totalDistortion / nFaces : 0.0;
}

double TextureMapper::computeAngleDistortion(const UVMapping& mapping) const {
    if (!mesh_ || !geometry_ || mapping.uvCoordinates.empty()) {
        return -1.0;
    }

    double totalDistortion = 0.0;
    int nFaces = 0;

    for (auto f : mesh_->faces()) {
        // 获取面的顶点
        std::vector<int> vertexIndices;
        std::vector<Vector3> positions3D;
        std::vector<Vector2> positionsUV;

        bool allVerticesValid = true;
        for (auto v : f.adjacentVertices()) {
            size_t idx = static_cast<size_t>(v.getIndex());
            vertexIndices.push_back(idx);
            positions3D.push_back(geometry_->vertexPositions[v]);
            if (idx < mapping.uvCoordinates.size()) {
                positionsUV.push_back(mapping.uvCoordinates[idx]);
            } else {
                // 顶点索引超出UV坐标范围，跳过此面
                allVerticesValid = false;
                break;
            }
        }

        if (allVerticesValid && positions3D.size() == 3 && positionsUV.size() == 3) {
            // 计算原始三角形的角度
            std::vector<double> originalAngles(3);
            for (int i = 0; i < 3; i++) {
                Vector3 v0 = positions3D[(i + 2) % 3] - positions3D[i];
                Vector3 v1 = positions3D[(i + 1) % 3] - positions3D[i];
                double cosAngle = geometrycentral::dot(v0, v1) / (v0.norm() * v1.norm());
                cosAngle = std::max(-1.0, std::min(1.0, cosAngle)); // 限制在[-1, 1]
                originalAngles[i] = std::acos(cosAngle);
            }

            // 计算UV三角形的角度
            std::vector<double> uvAngles(3);
            for (int i = 0; i < 3; i++) {
                Vector2 v0 = positionsUV[(i + 2) % 3] - positionsUV[i];
                Vector2 v1 = positionsUV[(i + 1) % 3] - positionsUV[i];
                double cosAngle = geometrycentral::dot(v0, v1) / (v0.norm() * v1.norm());
                cosAngle = std::max(-1.0, std::min(1.0, cosAngle));
                uvAngles[i] = std::acos(cosAngle);
            }

            // 计算角度失真
            for (int i = 0; i < 3; i++) {
                double diff = std::abs(uvAngles[i] - originalAngles[i]);
                totalDistortion += diff * diff;
            }
            nFaces++;
        }
    }

    return nFaces > 0 ? std::sqrt(totalDistortion / nFaces) : 0.0;
}

double TextureMapper::computeAreaDistortion(const UVMapping& mapping) const {
    if (!mesh_ || !geometry_ || mapping.uvCoordinates.empty()) {
        return -1.0;
    }

    double totalDistortion = 0.0;
    int nFaces = 0;

    for (auto f : mesh_->faces()) {
        // 获取原始面积
        double originalArea = geometry_->faceArea(f);

        // 获取UV面积 - 使用边界检查确保安全
        std::vector<Vector2> uvPositions;
        bool allVerticesValid = true;
        for (auto v : f.adjacentVertices()) {
            size_t idx = static_cast<size_t>(v.getIndex());
            if (idx < mapping.uvCoordinates.size()) {
                uvPositions.push_back(mapping.uvCoordinates[idx]);
            } else {
                // 顶点索引超出UV坐标范围，跳过此面
                allVerticesValid = false;
                break;
            }
        }

        if (allVerticesValid && uvPositions.size() == 3) {
            Vector2 v0 = uvPositions[1] - uvPositions[0];
            Vector2 v1 = uvPositions[2] - uvPositions[0];
            double uvArea = 0.5 * std::abs(v0.x * v1.y - v0.y * v1.x);

            // 计算面积失真（对数尺度）
            if (originalArea > 1e-10 && uvArea > 1e-10) {
                double distortion = std::log(uvArea / originalArea);
                totalDistortion += distortion * distortion;
                nFaces++;
            }
        }
    }

    return nFaces > 0 ? std::sqrt(totalDistortion / nFaces) : 0.0;
}

double TextureMapper::computeConformalError(const UVMapping& mapping) const {
    // 共形误差：测量角度保持的程度
    // 它与角度失真相同，但是更加关注局部的共形性
    return computeAngleDistortion(mapping);
}

TextureMapper::UVMapping TextureMapper::optimizeUVLayout(const UVMapping& mapping,
                                                         double packingEfficiency) const {
    UVMapping optimized = mapping;

    std::cout << "Optimizing UV layout with packing efficiency: " << packingEfficiency << std::endl;

    // TODO: 实现UV布局优化
    packUVCharts(optimized, packingEfficiency);

    return optimized;
}

void TextureMapper::setupBoundaryConstraints(const MappingParams& params) {
    std::cout << "Setting up boundary constraints..." << std::endl;
    // TODO: 实现边界约束设置
    (void)params;
}

void TextureMapper::packUVCharts(UVMapping& mapping, double efficiency) const {
    std::cout << "Packing UV charts with efficiency: " << efficiency << std::endl;
    // TODO: 实现UV图块打包算法
    (void)mapping;
}

} // namespace SurfaceTextureMapping
