/**
 * UV失真分析器实现
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include "uv_distortion_analyzer.h"
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>

using namespace geometrycentral;
using namespace geometrycentral::surface;

namespace SurfaceTextureMapping {

// ============================================================================
// 公共接口
// ============================================================================

void UVDistortionAnalyzer::setInput(
    std::shared_ptr<SurfaceMesh> mesh,
    std::shared_ptr<VertexPositionGeometry> geometry,
    const UVMapping& uvMapping)
{
    mesh_ = mesh;
    geometry_ = geometry;
    uvMapping_ = uvMapping;
}

std::vector<UVDistortionAnalyzer::FaceDistortion>
UVDistortionAnalyzer::computeAllDistortions()
{
    if (!mesh_ || !geometry_) {
        return {};
    }

    std::vector<FaceDistortion> distortions;
    distortions.reserve(mesh_->nFaces());

    for (Face f : mesh_->faces()) {
        distortions.push_back(computeFaceDistortion(f));
    }

    return distortions;
}

UVDistortionAnalyzer::FaceDistortion
UVDistortionAnalyzer::computeFaceDistortion(Face face)
{
    FaceDistortion result;

    // 计算雅可比矩阵
    result.jacobian = computeJacobian(face);

    // SVD分解得到奇异值
    result.sigmaValues = computeSVD(result.jacobian);
    result.sigmaMax = result.sigmaValues(0);
    result.sigmaMin = result.sigmaValues(1);

    // 计算失真度量
    result.conformalError = result.sigmaMax / result.sigmaMin;
    result.areaRatio = result.sigmaMax * result.sigmaMin;

    return result;
}

UVDistortionAnalyzer::GlobalDistortionStats
UVDistortionAnalyzer::computeGlobalStats(
    const std::vector<FaceDistortion>& faceDistortions)
{
    if (faceDistortions.empty()) {
        return GlobalDistortionStats{};
    }

    GlobalDistortionStats stats;

    // 收集各类度量
    std::vector<double> stretchValues;
    std::vector<double> conformalValues;
    std::vector<double> areaValues;

    stretchValues.reserve(faceDistortions.size());
    conformalValues.reserve(faceDistortions.size());
    areaValues.reserve(faceDistortions.size());

    for (const auto& dist : faceDistortions) {
        // 检查数值有效性
        if (std::isfinite(dist.sigmaMax) && std::isfinite(dist.sigmaMin)) {
            stretchValues.push_back(dist.sigmaMax);
            conformalValues.push_back(dist.conformalError);
            areaValues.push_back(dist.areaRatio);
        }
    }

    if (stretchValues.empty()) {
        return GlobalDistortionStats{};
    }

    // 拉伸失真统计
    stats.stretchMean = computeMean(stretchValues);
    stats.stretchStd = computeStd(stretchValues, stats.stretchMean);
    stats.stretchMax = *std::max_element(stretchValues.begin(), stretchValues.end());
    stats.stretchPercentile95 = computePercentile(stretchValues, 0.95);

    // 共形误差统计
    stats.conformalMean = computeMean(conformalValues);
    stats.conformalStd = computeStd(conformalValues, stats.conformalMean);
    stats.conformalMax = *std::max_element(conformalValues.begin(), conformalValues.end());
    stats.conformalPercentile95 = computePercentile(conformalValues, 0.95);

    // 面积失真统计
    stats.areaMean = computeMean(areaValues);
    stats.areaStd = computeStd(areaValues, stats.areaMean);
    stats.areaMax = *std::max_element(areaValues.begin(), areaValues.end());
    stats.areaPercentile95 = computePercentile(areaValues, 0.95);

    // 质量判定
    stats.passStretchThreshold = stats.stretchMax <= stretchThreshold_;
    stats.passConformalThreshold = stats.conformalMax <= conformalThreshold_;
    stats.passOverallQuality = stats.passStretchThreshold && stats.passConformalThreshold;

    return stats;
}

UVDistortionAnalyzer::ColorMap
UVDistortionAnalyzer::generateColorMap(
    const std::vector<FaceDistortion>& faceDistortions,
    const std::string& metricType)
{
    ColorMap colorMap;
    colorMap.metricName = metricType;
    colorMap.faceColors.reserve(faceDistortions.size());

    if (faceDistortions.empty()) {
        return colorMap;
    }

    // 提取对应度量的值
    std::vector<double> values;
    values.reserve(faceDistortions.size());

    for (const auto& dist : faceDistortions) {
        if (metricType == "Stretch") {
            values.push_back(dist.sigmaMax);
        } else if (metricType == "Conformal") {
            values.push_back(dist.conformalError);
        } else if (metricType == "Area") {
            values.push_back(dist.areaRatio);
        } else {
            values.push_back(dist.sigmaMax); // 默认使用拉伸失真
        }
    }

    // 计算值域
    colorMap.minValue = *std::min_element(values.begin(), values.end());
    colorMap.maxValue = *std::max_element(values.begin(), values.end());

    // 生成颜色
    for (double value : values) {
        colorMap.faceColors.push_back(
            valueToColor(value, colorMap.minValue, colorMap.maxValue)
        );
    }

    return colorMap;
}

std::vector<size_t> UVDistortionAnalyzer::markHighDistortionRegions(
    const std::vector<FaceDistortion>& faceDistortions,
    double threshold)
{
    std::vector<size_t> highDistortionFaces;

    for (size_t i = 0; i < faceDistortions.size(); ++i) {
        const auto& dist = faceDistortions[i];
        if (dist.sigmaMax > threshold) {
            highDistortionFaces.push_back(i);
        }
    }

    return highDistortionFaces;
}

void UVDistortionAnalyzer::exportDistortionReport(
    const std::string& filename,
    const GlobalDistortionStats& stats,
    const std::vector<FaceDistortion>& faceDistortions)
{
    std::ofstream outFile(filename);
    if (!outFile) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return;
    }

    // 写入全局统计
    outFile << "=== UV失真分析报告 ===" << std::endl;
    outFile << std::endl;
    outFile << "拉伸失真统计:" << std::endl;
    outFile << "  平均值: " << stats.stretchMean << std::endl;
    outFile << "  标准差: " << stats.stretchStd << std::endl;
    outFile << "  最大值: " << stats.stretchMax << std::endl;
    outFile << "  95百分位: " << stats.stretchPercentile95 << std::endl;
    outFile << std::endl;

    outFile << "共形误差统计:" << std::endl;
    outFile << "  平均值: " << stats.conformalMean << std::endl;
    outFile << "  标准差: " << stats.conformalStd << std::endl;
    outFile << "  最大值: " << stats.conformalMax << std::endl;
    outFile << "  95百分位: " << stats.conformalPercentile95 << std::endl;
    outFile << std::endl;

    outFile << "面积失真统计:" << std::endl;
    outFile << "  平均值: " << stats.areaMean << std::endl;
    outFile << "  标准差: " << stats.areaStd << std::endl;
    outFile << "  最大值: " << stats.areaMax << std::endl;
    outFile << "  95百分位: " << stats.areaPercentile95 << std::endl;
    outFile << std::endl;

    outFile << "质量判定:" << std::endl;
    outFile << "  拉伸阈值 (" << stretchThreshold_ << "): "
            << (stats.passStretchThreshold ? "通过" : "失败") << std::endl;
    outFile << "  共形阈值 (" << conformalThreshold_ << "): "
            << (stats.passConformalThreshold ? "通过" : "失败") << std::endl;
    outFile << "  总体质量: "
            << (stats.passOverallQuality ? "通过" : "失败") << std::endl;

    outFile.close();
    std::cout << "失真报告已导出到: " << filename << std::endl;
}

// ============================================================================
// 私有实现 - 核心算法
// ============================================================================

Eigen::Matrix2d UVDistortionAnalyzer::computeJacobian(Face face)
{
    // 获取三角形的三个顶点
    auto vertices = face.adjacentVertices();
    auto it = vertices.begin();
    Vertex v0 = *it;
    ++it;
    Vertex v1 = *it;
    ++it;
    Vertex v2 = *it;

    // 获取3D坐标
    Vector3 p0 = geometry_->vertexPositions[v0];
    Vector3 p1 = geometry_->vertexPositions[v1];
    Vector3 p2 = geometry_->vertexPositions[v2];

    // 获取UV坐标
    size_t idx0 = v0.getIndex();
    size_t idx1 = v1.getIndex();
    size_t idx2 = v2.getIndex();

    if (idx0 >= uvMapping_.uvCoordinates.size() ||
        idx1 >= uvMapping_.uvCoordinates.size() ||
        idx2 >= uvMapping_.uvCoordinates.size()) {
        // UV坐标缺失，返回单位矩阵
        return Eigen::Matrix2d::Identity();
    }

    Vector2 uv0 = uvMapping_.uvCoordinates[idx0];
    Vector2 uv1 = uvMapping_.uvCoordinates[idx1];
    Vector2 uv2 = uvMapping_.uvCoordinates[idx2];

    // 构建3D边向量
    Vector3 e1_3d = p1 - p0;
    Vector3 e2_3d = p2 - p0;

    // 构建UV边向量
    Vector2 e1_uv = uv1 - uv0;
    Vector2 e2_uv = uv2 - uv0;

    // 计算3D三角形的局部坐标系
    // 检查三角形是否接近平面 (判断法向量是否接近Z轴)
    Vector3 normal = cross(e1_3d, e2_3d).normalize();
    bool isNearXYPlane = (std::abs(normal.x) < 1e-6 &&
                          std::abs(normal.y) < 1e-6 &&
                          std::abs(normal.z) > 0.999);

    Eigen::Vector2d e1_2d, e2_2d;

    if (isNearXYPlane) {
        // 对于XY平面上的三角形，直接使用全局XY坐标
        // 这样可以保证完美映射(UV=XY)得到单位Jacobian
        e1_2d << e1_3d.x, e1_3d.y;
        e2_2d << e2_3d.x, e2_3d.y;
    } else {
        // 对于一般曲面，使用局部坐标系
        // X轴: e1方向
        // Y轴: e1和e2张成平面内垂直于X的方向
        Vector3 xAxis = e1_3d.normalize();
        Vector3 yAxis = cross(normal, xAxis).normalize();

        // 将3D边投影到局部2D坐标系
        e1_2d << dot(e1_3d, xAxis), dot(e1_3d, yAxis);
        e2_2d << dot(e2_3d, xAxis), dot(e2_3d, yAxis);
    }

    // 构建从局部2D到UV的映射矩阵
    // [e1_uv, e2_uv] = J * [e1_2d, e2_2d]
    // J = [e1_uv, e2_uv] * [e1_2d, e2_2d]^-1

    Eigen::Matrix2d E_2d;
    E_2d.col(0) = e1_2d;
    E_2d.col(1) = e2_2d;

    Eigen::Matrix2d E_uv;
    E_uv << e1_uv.x, e2_uv.x,
            e1_uv.y, e2_uv.y;

    // 检查退化情况
    double det = E_2d.determinant();
    if (std::abs(det) < 1e-10) {
        // 退化三角形，返回单位矩阵
        return Eigen::Matrix2d::Identity();
    }

    // 计算雅可比矩阵
    Eigen::Matrix2d jacobian = E_uv * E_2d.inverse();

    return jacobian;
}

Eigen::Vector2d UVDistortionAnalyzer::computeSVD(const Eigen::Matrix2d& jacobian)
{
    // 使用Eigen库进行SVD分解
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(
        jacobian,
        Eigen::ComputeFullU | Eigen::ComputeFullV
    );

    Eigen::Vector2d singularValues = svd.singularValues();

    // 确保 σ_max >= σ_min
    double sigma_max = std::max(singularValues(0), singularValues(1));
    double sigma_min = std::min(singularValues(0), singularValues(1));

    // 防止除零
    if (sigma_min < 1e-10) {
        sigma_min = 1e-10;
    }

    return Eigen::Vector2d(sigma_max, sigma_min);
}

// ============================================================================
// 私有实现 - 统计函数
// ============================================================================

double UVDistortionAnalyzer::computeMean(const std::vector<double>& values)
{
    if (values.empty()) {
        return 0.0;
    }

    double sum = std::accumulate(values.begin(), values.end(), 0.0);
    return sum / values.size();
}

double UVDistortionAnalyzer::computeStd(const std::vector<double>& values, double mean)
{
    if (values.size() <= 1) {
        return 0.0;
    }

    double variance = 0.0;
    for (double value : values) {
        double diff = value - mean;
        variance += diff * diff;
    }
    variance /= (values.size() - 1);

    return std::sqrt(variance);
}

double UVDistortionAnalyzer::computePercentile(
    const std::vector<double>& values,
    double percentile)
{
    if (values.empty()) {
        return 0.0;
    }

    // 复制并排序
    std::vector<double> sorted = values;
    std::sort(sorted.begin(), sorted.end());

    // 计算索引
    double index = percentile * (sorted.size() - 1);
    size_t lower = static_cast<size_t>(std::floor(index));
    size_t upper = static_cast<size_t>(std::ceil(index));

    if (lower == upper) {
        return sorted[lower];
    }

    // 线性插值
    double weight = index - lower;
    return sorted[lower] * (1.0 - weight) + sorted[upper] * weight;
}

// ============================================================================
// 私有实现 - 颜色映射
// ============================================================================

Eigen::Vector3d UVDistortionAnalyzer::valueToColor(
    double value,
    double minVal,
    double maxVal)
{
    // 归一化到 [0, 1]
    double normalized = 0.0;
    if (maxVal > minVal) {
        normalized = (value - minVal) / (maxVal - minVal);
    }

    // 限制在有效范围
    normalized = std::max(0.0, std::min(1.0, normalized));

    // Rainbow色谱映射
    // 0.0 -> 蓝色 (低失真)
    // 0.5 -> 绿/黄色 (中等失真)
    // 1.0 -> 红色 (高失真)

    double r, g, b;

    if (normalized < 0.25) {
        // 蓝 -> 青
        double t = normalized / 0.25;
        r = 0.0;
        g = t;
        b = 1.0;
    } else if (normalized < 0.5) {
        // 青 -> 绿
        double t = (normalized - 0.25) / 0.25;
        r = 0.0;
        g = 1.0;
        b = 1.0 - t;
    } else if (normalized < 0.75) {
        // 绿 -> 黄
        double t = (normalized - 0.5) / 0.25;
        r = t;
        g = 1.0;
        b = 0.0;
    } else {
        // 黄 -> 红
        double t = (normalized - 0.75) / 0.25;
        r = 1.0;
        g = 1.0 - t;
        b = 0.0;
    }

    return Eigen::Vector3d(r, g, b);
}

std::vector<Eigen::Vector3d> UVDistortionAnalyzer::smoothVertexColors(
    const std::vector<Eigen::Vector3d>& faceColors)
{
    if (!mesh_ || faceColors.empty()) {
        return {};
    }

    std::vector<Eigen::Vector3d> vertexColors(mesh_->nVertices(), Eigen::Vector3d::Zero());
    std::vector<int> vertexCounts(mesh_->nVertices(), 0);

    // 从面颜色平均到顶点
    size_t faceIdx = 0;
    for (Face f : mesh_->faces()) {
        if (faceIdx >= faceColors.size()) {
            break;
        }

        const Eigen::Vector3d& faceColor = faceColors[faceIdx];

        for (Vertex v : f.adjacentVertices()) {
            size_t vIdx = v.getIndex();
            vertexColors[vIdx] += faceColor;
            vertexCounts[vIdx]++;
        }

        faceIdx++;
    }

    // 归一化
    for (size_t i = 0; i < vertexColors.size(); ++i) {
        if (vertexCounts[i] > 0) {
            vertexColors[i] /= vertexCounts[i];
        }
    }

    return vertexColors;
}

} // namespace SurfaceTextureMapping