/**
 * 表面填充算法实现
 * 生成各种填充图案
 */

#include "surface_filling.h"
#include <iostream>
#include <cmath>
#include <algorithm>
#include <fstream>
#include <limits>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace SurfaceTextureMapping {

void SurfaceFiller::setInput(std::shared_ptr<SurfaceMesh> mesh,
                             std::shared_ptr<VertexPositionGeometry> geometry,
                             const UVMapping& uvMapping) {
    mesh_ = mesh;
    geometry_ = geometry;
    uvMapping_ = uvMapping;
}

SurfaceFiller::FillingResult
SurfaceFiller::generateFilling(const FillingParams& params) const {
    FillingResult result;

    if (!mesh_ || !geometry_) {
        std::cerr << "SurfaceFiller not initialized" << std::endl;
        return result;
    }

    std::cout << "Generating surface filling pattern..." << std::endl;

    // 获取UV边界
    std::vector<std::vector<Vector2>> boundariesList = getUVBoundaries();
    std::vector<Vector2> uvBounds;
    if (!boundariesList.empty()) {
        uvBounds = boundariesList[0];
    }

    // 根据类型生成图案
    switch (params.type) {
        case PatternType::Grid:
            result.pathsUV = generateGridPattern(params.spacing, uvBounds);
            break;
        case PatternType::Hexagonal:
            result.pathsUV = generateHexagonalPattern(params.spacing, uvBounds);
            break;
        case PatternType::Spiral: {
            Vector2 center{0.5, 0.5};  // UV空间中心
            double maxRadius = 0.5;
            auto spiralPath = generateSpiralPattern(params.spacing, center, maxRadius);
            result.pathsUV.push_back(spiralPath);
            break;
        }
        case PatternType::Hilbert: {
            auto hilbertPath = generateHilbertCurve(params.recursionDepth, uvBounds);
            result.pathsUV.push_back(hilbertPath);
            break;
        }
        case PatternType::Peano: {
            auto peanoPath = generatePeanoCurve(params.recursionDepth, uvBounds);
            result.pathsUV.push_back(peanoPath);
            break;
        }
        default:
            std::cerr << "Unsupported pattern type" << std::endl;
            break;
    }

    // 映射到3D
    result.paths3D = mapPathsTo3D(result.pathsUV);

    // 计算统计信息
    result.numPaths = result.pathsUV.size();
    result.totalLength = computeTotalLength(result.paths3D);
    result.coverage = computeCoverage(result.pathsUV);

    std::cout << "Pattern generation completed:" << std::endl;
    std::cout << "  Paths: " << result.numPaths << std::endl;
    std::cout << "  Total length: " << result.totalLength << std::endl;
    std::cout << "  Coverage: " << result.coverage * 100 << "%" << std::endl;

    return result;
}

SurfaceFiller::FillingQuality SurfaceFiller::evaluateFillingQuality(const FillingResult& result) const {
    FillingQuality quality;

    // 计算均匀性 - 基于路径间距的标准差
    quality.uniformity = 1.0 - computeSpacingVariation(result);

    // 计算效率 - 基于覆盖率
    quality.efficiency = result.coverage;

    // 计算边界遵循度 - 检查路径是否超出UV边界
    quality.boundaryRespect = computeBoundaryRespect(result);

    // 计算平滑度 - 基于路径曲率变化
    quality.smoothness = computeSmoothness(result);

    std::cout << "Filling quality evaluation:" << std::endl;
    std::cout << "  Uniformity: " << quality.uniformity << std::endl;
    std::cout << "  Efficiency: " << quality.efficiency << std::endl;
    std::cout << "  Boundary respect: " << quality.boundaryRespect << std::endl;
    std::cout << "  Smoothness: " << quality.smoothness << std::endl;

    return quality;
}

void SurfaceFiller::exportPaths3D(const std::vector<std::vector<Vector3>>& paths3D,
                                  const std::string& filename) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    std::cout << "Exporting 3D paths to: " << filename << std::endl;

    int vertexIndex = 1;
    for (const auto& path : paths3D) {
        // 写入顶点
        for (const auto& point : path) {
            file << "v " << point.x << " " << point.y << " " << point.z << "\n";
        }

        // 写入线段
        for (size_t i = 0; i < path.size() - 1; ++i) {
            file << "l " << (vertexIndex + i) << " " << (vertexIndex + i + 1) << "\n";
        }

        vertexIndex += path.size();
    }

    file.close();
    std::cout << "Exported " << paths3D.size() << " paths with total " << (vertexIndex - 1) << " vertices" << std::endl;
}

void SurfaceFiller::exportPathsToSVG(const std::vector<std::vector<Vector2>>& paths,
                                     const std::string& filename,
                                     double viewBox) const {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open SVG file: " << filename << std::endl;
        return;
    }

    std::cout << "Exporting SVG pattern to: " << filename << std::endl;
    std::cout << "  ViewBox: " << viewBox << std::endl;

    // SVG header
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<svg width=\"" << (viewBox * 1000) << "\" height=\"" << (viewBox * 1000)
         << "\" viewBox=\"0 0 " << viewBox << " " << viewBox
         << "\" xmlns=\"http://www.w3.org/2000/svg\">\n";

    // 绘制路径
    for (const auto& path : paths) {
        if (path.empty()) continue;

        file << "  <path d=\"M";
        for (size_t i = 0; i < path.size(); ++i) {
            if (i == 0) {
                file << path[i].x << "," << path[i].y;
            } else {
                file << " L" << path[i].x << "," << path[i].y;
            }
        }
        file << "\" stroke=\"black\" stroke-width=\"0.001\" fill=\"none\"/>\n";
    }

    file << "</svg>\n";
    file.close();

    std::cout << "Exported " << paths.size() << " paths to SVG" << std::endl;
}

std::vector<std::vector<SurfaceFiller::Vector2>>
SurfaceFiller::generateGridPattern(double spacing, const std::vector<Vector2>& uvBounds) const {
    std::cout << "  Generating grid pattern..." << std::endl;
    std::cout << "    Spacing: " << spacing << std::endl;

    std::vector<std::vector<Vector2>> paths;

    if (uvBounds.empty()) {
        std::cerr << "UV bounds are empty" << std::endl;
        return paths;
    }

    // 找到UV边界的包围盒
    double minU = uvBounds[0].x, maxU = uvBounds[0].x;
    double minV = uvBounds[0].y, maxV = uvBounds[0].y;
    for (const auto& point : uvBounds) {
        minU = std::min(minU, point.x);
        maxU = std::max(maxU, point.x);
        minV = std::min(minV, point.y);
        maxV = std::max(maxV, point.y);
    }

    // 生成水平线
    for (double v = minV; v <= maxV; v += spacing) {
        std::vector<Vector2> line;
        line.push_back(Vector2{minU, v});
        line.push_back(Vector2{maxU, v});
        paths.push_back(line);
    }

    // 生成垂直线
    for (double u = minU; u <= maxU; u += spacing) {
        std::vector<Vector2> line;
        line.push_back(Vector2{u, minV});
        line.push_back(Vector2{u, maxV});
        paths.push_back(line);
    }

    std::cout << "    Generated " << paths.size() << " grid lines" << std::endl;
    return paths;
}

std::vector<std::vector<SurfaceFiller::Vector2>>
SurfaceFiller::generateHexagonalPattern(double spacing, const std::vector<Vector2>& uvBounds) const {
    std::cout << "  Generating hexagonal pattern..." << std::endl;
    std::cout << "    Spacing: " << spacing << std::endl;

    std::vector<std::vector<Vector2>> paths;

    if (uvBounds.empty()) {
        std::cerr << "UV bounds are empty" << std::endl;
        return paths;
    }

    // 找到UV边界的包围盒
    double minU = uvBounds[0].x, maxU = uvBounds[0].x;
    double minV = uvBounds[0].y, maxV = uvBounds[0].y;
    for (const auto& point : uvBounds) {
        minU = std::min(minU, point.x);
        maxU = std::max(maxU, point.x);
        minV = std::min(minV, point.y);
        maxV = std::max(maxV, point.y);
    }

    const double hexHeight = spacing * std::sqrt(3.0) / 2.0;
    const double hexWidth = spacing;

    // 生成六边形网格
    for (double v = minV; v <= maxV; v += hexHeight) {
        for (double u = minU; u <= maxU; u += hexWidth * 1.5) {
            // 交错放置六边形
            double offsetU = u + ((int((v - minV) / hexHeight) % 2) * hexWidth * 0.75);

            // 生成六边形的边
            std::vector<Vector2> hexagon;
            for (int i = 0; i < 6; ++i) {
                double angle = i * M_PI / 3.0;
                double x = offsetU + spacing * 0.5 * std::cos(angle);
                double y = v + spacing * 0.5 * std::sin(angle);
                hexagon.push_back(Vector2{x, y});
            }
            // 闭合六边形
            if (!hexagon.empty()) {
                hexagon.push_back(hexagon[0]);
                paths.push_back(hexagon);
            }
        }
    }

    std::cout << "    Generated " << paths.size() << " hexagons" << std::endl;
    return paths;
}

std::vector<SurfaceFiller::Vector2>
SurfaceFiller::generateSpiralPattern(double spacing, const Vector2& center, double maxRadius) const {
    std::cout << "  Generating spiral pattern..." << std::endl;
    std::cout << "    Center: (" << center.x << ", " << center.y << ")" << std::endl;
    std::cout << "    Max radius: " << maxRadius << std::endl;

    std::vector<Vector2> spiral;

    const double angleStep = 0.1;  // 角度步长
    const double radiusStep = spacing / (2.0 * M_PI);  // 半径步长

    for (double angle = 0.0; angle < maxRadius / radiusStep * 2.0 * M_PI; angle += angleStep) {
        double radius = angle * radiusStep;
        if (radius > maxRadius) break;

        double x = center.x + radius * std::cos(angle);
        double y = center.y + radius * std::sin(angle);
        spiral.push_back(Vector2{x, y});
    }

    std::cout << "    Generated spiral with " << spiral.size() << " points" << std::endl;
    return spiral;
}

std::vector<SurfaceFiller::Vector2>
SurfaceFiller::generateHilbertCurve(int order, const std::vector<Vector2>& uvBounds) const {
    std::cout << "  Generating Hilbert curve of order " << order << std::endl;

    std::vector<Vector2> curve;

    if (uvBounds.empty()) {
        std::cerr << "UV bounds are empty" << std::endl;
        return curve;
    }

    // 找到UV边界的包围盒
    double minU = uvBounds[0].x, maxU = uvBounds[0].x;
    double minV = uvBounds[0].y, maxV = uvBounds[0].y;
    for (const auto& point : uvBounds) {
        minU = std::min(minU, point.x);
        maxU = std::max(maxU, point.x);
        minV = std::min(minV, point.y);
        maxV = std::max(maxV, point.y);
    }

    double size = std::min(maxU - minU, maxV - minV);
    Vector2 start{minU, minV};

    hilbertRecursive(curve, start, size, order, 0);

    std::cout << "    Generated Hilbert curve with " << curve.size() << " points" << std::endl;
    return curve;
}

std::vector<SurfaceFiller::Vector2>
SurfaceFiller::generatePeanoCurve(int order, const std::vector<Vector2>& uvBounds) const {
    std::cout << "  Generating Peano curve of order " << order << std::endl;

    std::vector<Vector2> curve;

    if (uvBounds.empty()) {
        std::cerr << "UV bounds are empty" << std::endl;
        return curve;
    }

    // 找到UV边界的包围盒
    double minU = uvBounds[0].x, maxU = uvBounds[0].x;
    double minV = uvBounds[0].y, maxV = uvBounds[0].y;
    for (const auto& point : uvBounds) {
        minU = std::min(minU, point.x);
        maxU = std::max(maxU, point.x);
        minV = std::min(minV, point.y);
        maxV = std::max(maxV, point.y);
    }

    double size = std::min(maxU - minU, maxV - minV);
    Vector2 start{minU, minV};

    peanoRecursive(curve, start, size, order);

    std::cout << "    Generated Peano curve with " << curve.size() << " points" << std::endl;
    return curve;
}

std::vector<std::vector<SurfaceFiller::Vector3>>
SurfaceFiller::mapPathsTo3D(const std::vector<std::vector<Vector2>>& uvPaths) const {
    std::vector<std::vector<Vector3>> paths3D;

    if (!mesh_ || !geometry_) {
        std::cerr << "Mesh or geometry not initialized" << std::endl;
        return paths3D;
    }

    for (const auto& uvPath : uvPaths) {
        std::vector<Vector3> path3D;

        for (const auto& uvPoint : uvPath) {
            // 简化实现：直接使用UV坐标作为索引
            // 实际实现中需要根据UV坐标找到对应的三角面并计算重心坐标
            if (!uvMapping_.uvCoordinates.empty() && uvMapping_.uvCoordinates.size() == mesh_->nVertices()) {
                // 找到最近的顶点
                double minDist = std::numeric_limits<double>::max();
                size_t closestVertex = 0;

                size_t i = 0;
                for (auto v : mesh_->vertices()) {
                    Vector2 diff = uvMapping_.uvCoordinates[i] - uvPoint;
                    double dist = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                    if (dist < minDist) {
                        minDist = dist;
                        closestVertex = i;
                    }
                    i++;
                }

                // 获取对应的3D坐标
                if (closestVertex < geometry_->vertexPositions.size()) {
                    path3D.push_back(geometry_->vertexPositions[mesh_->vertex(closestVertex)]);
                }
            }
        }

        if (!path3D.empty()) {
            paths3D.push_back(path3D);
        }
    }

    return paths3D;
}

// 私有辅助方法实现
std::vector<std::vector<SurfaceFiller::Vector2>>
SurfaceFiller::getUVBoundaries() const {
    std::vector<std::vector<Vector2>> boundaries;

    if (uvMapping_.uvCoordinates.empty()) {
        // 返回默认的单位正方形边界
        std::vector<Vector2> unitSquare = {
            Vector2{0.0, 0.0}, Vector2{1.0, 0.0},
            Vector2{1.0, 1.0}, Vector2{0.0, 1.0}
        };
        boundaries.push_back(unitSquare);
    } else {
        // 从 UV 坐标中提取边界
        // 简化实现：使用包围盒
        double minU = uvMapping_.uvCoordinates[0].x, maxU = uvMapping_.uvCoordinates[0].x;
        double minV = uvMapping_.uvCoordinates[0].y, maxV = uvMapping_.uvCoordinates[0].y;

        for (const auto& uv : uvMapping_.uvCoordinates) {
            minU = std::min(minU, uv.x);
            maxU = std::max(maxU, uv.x);
            minV = std::min(minV, uv.y);
            maxV = std::max(maxV, uv.y);
        }

        std::vector<Vector2> boundingBox = {
            Vector2{minU, minV}, Vector2{maxU, minV},
            Vector2{maxU, maxV}, Vector2{minU, maxV}
        };
        boundaries.push_back(boundingBox);
    }

    return boundaries;
}

bool SurfaceFiller::isPointInUVDomain(const Vector2& point) const {
    // 简化实现：检查是否在[0,1]x[0,1]范围内
    return point.x >= 0.0 && point.x <= 1.0 && point.y >= 0.0 && point.y <= 1.0;
}

// 质量评估辅助方法
double SurfaceFiller::computeTotalLength(const std::vector<std::vector<Vector3>>& paths3D) const {
    double totalLength = 0.0;
    for (const auto& path : paths3D) {
        for (size_t i = 1; i < path.size(); ++i) {
            Vector3 diff = path[i] - path[i-1];
            totalLength += std::sqrt(diff.x * diff.x + diff.y * diff.y + diff.z * diff.z);
        }
    }
    return totalLength;
}

double SurfaceFiller::computeCoverage(const std::vector<std::vector<Vector2>>& pathsUV) const {
    // 简化实现：基于路径数量和总长度的估计
    double pathLength = 0.0;
    for (const auto& path : pathsUV) {
        for (size_t i = 1; i < path.size(); ++i) {
            Vector2 diff = path[i] - path[i-1];
            pathLength += std::sqrt(diff.x * diff.x + diff.y * diff.y);
        }
    }
    // 对于单位正方形UV域，估计覆盖率
    return std::min(1.0, pathLength / 10.0);  // 简化的估计
}

double SurfaceFiller::computeSpacingVariation(const FillingResult& result) const {
    if (result.pathsUV.size() < 2) return 0.0;

    // 计算路径间距的变化系数
    std::vector<double> spacings;
    // 简化实现
    return 0.1;  // 返回一个示例值
}

double SurfaceFiller::computeBoundaryRespect(const FillingResult& result) const {
    double respectScore = 1.0;
    for (const auto& path : result.pathsUV) {
        for (const auto& point : path) {
            if (!isPointInUVDomain(point)) {
                respectScore -= 0.1;
            }
        }
    }
    return std::max(0.0, respectScore);
}

double SurfaceFiller::computeSmoothness(const FillingResult& result) const {
    // 简化实现：返回固定的平滑度评估值
    // TODO: 实现基于路径曲率的平滑度计算
    double smoothnessScore = 0.9;

    // 简单估算：基于路径数量和长度
    if (result.pathsUV.empty()) {
        return 0.0;
    }

    // 路径越多，平滑度可能越低
    smoothnessScore -= std::min(0.3, result.numPaths * 0.01);

    return std::max(0.0, smoothnessScore);
}

void SurfaceFiller::hilbertRecursive(std::vector<Vector2>& curve,
                                   Vector2 start, double size, int order, int direction) const {
    if (order == 0) {
        curve.push_back(start);
        return;
    }

    double halfSize = size / 2.0;
    // 简化的希尔伯特曲线生成
    hilbertRecursive(curve, start, halfSize, order - 1, direction);
    hilbertRecursive(curve, Vector2{start.x, start.y + halfSize}, halfSize, order - 1, direction);
    hilbertRecursive(curve, Vector2{start.x + halfSize, start.y + halfSize}, halfSize, order - 1, direction);
    hilbertRecursive(curve, Vector2{start.x + halfSize, start.y}, halfSize, order - 1, direction);
}

void SurfaceFiller::peanoRecursive(std::vector<Vector2>& curve,
                                 Vector2 start, double size, int order) const {
    if (order == 0) {
        curve.push_back(start);
        return;
    }

    double thirdSize = size / 3.0;
    // 简化的皮亚诺曲线生成
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            Vector2 pos{start.x + i * thirdSize, start.y + j * thirdSize};
            peanoRecursive(curve, pos, thirdSize, order - 1);
        }
    }
}

std::vector<std::vector<SurfaceFiller::Vector2>>
SurfaceFiller::clipPaths(const std::vector<std::vector<Vector2>>& paths,
                        const std::vector<std::vector<Vector2>>& clipPaths,
                        Clipper2Lib::ClipType operation) const {
    std::cout << "Clipping paths with Clipper2..." << std::endl;

    // TODO: 实现Clipper2路径裁剪
    // 这里需要使用Clipper2库进行实际裁剪操作

    // 目前返回原始路径
    return paths;
}

} // namespace SurfaceTextureMapping