#pragma once

/**
 * UV失真分析器
 * 计算UV映射的各种失真度量，包括拉伸失真、共形误差、面积失真
 * 基于雅可比矩阵的SVD分解进行计算
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <Eigen/SVD>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "texture_mapping.h"

namespace SurfaceTextureMapping {

/**
 * UV失真分析器类
 *
 * 核心算法：
 * 对每个三角形，计算从3D到UV的雅可比矩阵J，然后进行SVD分解：
 * J = U * Σ * V^T
 * 其中 Σ = diag(σ_max, σ_min) 是奇异值
 *
 * 失真度量：
 * - 拉伸失真: σ_max (最大拉伸倍数)
 * - 共形误差: QC = σ_max / σ_min (各向异性度)
 * - 面积失真: A_ratio = (σ_max * σ_min) = det(J)
 */
class UVDistortionAnalyzer {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector2 = geometrycentral::Vector2;
    using Vector3 = geometrycentral::Vector3;
    using Face = geometrycentral::surface::Face;
    using UVMapping = TextureMapper::UVMapping;

    /**
     * 每个三角形的失真度量
     */
    struct FaceDistortion {
        double sigmaMax;           // 最大奇异值 (最大拉伸)
        double sigmaMin;           // 最小奇异值 (最小拉伸)
        double conformalError;     // 共形误差 QC = σ_max/σ_min
        double areaRatio;          // 面积比 = σ_max * σ_min
        Eigen::Matrix2d jacobian;  // 雅可比矩阵
        Eigen::Vector2d sigmaValues; // 奇异值向量
    };

    /**
     * 全局失真统计
     */
    struct GlobalDistortionStats {
        // 拉伸失真统计
        double stretchMean;
        double stretchStd;
        double stretchMax;
        double stretchPercentile95;

        // 共形误差统计
        double conformalMean;
        double conformalStd;
        double conformalMax;
        double conformalPercentile95;

        // 面积失真统计
        double areaMean;
        double areaStd;
        double areaMax;
        double areaPercentile95;

        // 质量判定
        bool passStretchThreshold;    // 拉伸失真 < 阈值
        bool passConformalThreshold;  // 共形误差 < 阈值
        bool passOverallQuality;      // 总体质量判定
    };

    /**
     * 颜色编码用于可视化
     */
    struct ColorMap {
        std::vector<Eigen::Vector3d> faceColors;  // 每个面的RGB颜色
        double minValue;
        double maxValue;
        std::string metricName;  // "Stretch", "Conformal", "Area"
    };

    UVDistortionAnalyzer() = default;
    ~UVDistortionAnalyzer() = default;

    /**
     * 设置输入网格和UV映射
     * @param mesh 3D网格
     * @param geometry 几何信息
     * @param uvMapping UV坐标映射
     */
    void setInput(std::shared_ptr<SurfaceMesh> mesh,
                  std::shared_ptr<VertexPositionGeometry> geometry,
                  const UVMapping& uvMapping);

    /**
     * 计算所有三角形的失真度量
     * @return 每个面的失真数据
     */
    std::vector<FaceDistortion> computeAllDistortions();

    /**
     * 计算单个三角形的失真
     * @param face 目标三角形
     * @return 失真度量
     */
    FaceDistortion computeFaceDistortion(Face face);

    /**
     * 计算全局统计信息
     * @param faceDistortions 所有面的失真数据
     * @return 全局统计
     */
    GlobalDistortionStats computeGlobalStats(
        const std::vector<FaceDistortion>& faceDistortions);

    /**
     * 生成失真可视化颜色映射
     * @param faceDistortions 失真数据
     * @param metricType "Stretch", "Conformal", "Area"
     * @return 颜色映射
     */
    ColorMap generateColorMap(
        const std::vector<FaceDistortion>& faceDistortions,
        const std::string& metricType);

    /**
     * 设置质量阈值
     */
    void setStretchThreshold(double threshold) { stretchThreshold_ = threshold; }
    void setConformalThreshold(double threshold) { conformalThreshold_ = threshold; }
    void setAreaThreshold(double threshold) { areaThreshold_ = threshold; }

    /**
     * 导出失真报告
     * @param filename 输出文件名
     * @param stats 全局统计
     * @param faceDistortions 详细失真数据
     */
    void exportDistortionReport(
        const std::string& filename,
        const GlobalDistortionStats& stats,
        const std::vector<FaceDistortion>& faceDistortions);

    /**
     * 标记高失真区域 (用于迭代返修)
     * @param faceDistortions 失真数据
     * @param threshold 失真阈值
     * @return 需要返修的面索引列表
     */
    std::vector<size_t> markHighDistortionRegions(
        const std::vector<FaceDistortion>& faceDistortions,
        double threshold);

private:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
    UVMapping uvMapping_;

    // 质量阈值
    double stretchThreshold_ = 2.0;      // 最大拉伸 < 2.0 (100%拉伸)
    double conformalThreshold_ = 1.5;    // 共形误差 < 1.5
    double areaThreshold_ = 2.0;         // 面积变化 < 2.0

    /**
     * 计算三角形的雅可比矩阵
     * J = [∂u/∂x  ∂u/∂y]
     *     [∂v/∂x  ∂v/∂y]
     */
    Eigen::Matrix2d computeJacobian(Face face);

    /**
     * 对雅可比矩阵进行SVD分解
     * @return (σ_max, σ_min)
     */
    Eigen::Vector2d computeSVD(const Eigen::Matrix2d& jacobian);

    /**
     * 计算统计量 (mean, std, percentile)
     */
    double computeMean(const std::vector<double>& values);
    double computeStd(const std::vector<double>& values, double mean);
    double computePercentile(const std::vector<double>& values, double percentile);

    /**
     * 颜色映射函数 (值 -> RGB)
     * 使用Rainbow色谱: 蓝(低失真) -> 绿 -> 黄 -> 红(高失真)
     */
    Eigen::Vector3d valueToColor(double value, double minVal, double maxVal);

    /**
     * 顶点颜色平滑 (从面颜色插值到顶点)
     */
    std::vector<Eigen::Vector3d> smoothVertexColors(
        const std::vector<Eigen::Vector3d>& faceColors);
};

} // namespace SurfaceTextureMapping