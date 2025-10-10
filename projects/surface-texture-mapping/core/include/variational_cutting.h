#pragma once

#include <vector>
#include <optional>
#include <memory>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh.h"

namespace SurfaceTextureMapping {

/**
 * 变分切缝类
 * 实现基于Yamabe方程形状导数的变分表面切割
 */
class VariationalCutter {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector3 = geometrycentral::Vector3;

    struct CuttingParams {
        double lengthRegularization = 0.1;    // 长度正则化权重
        double smoothRegularization = 0.05;   // 平滑正则化权重
        double timeStep = 0.01;               // 时间步长
        int maxIterations = 100;              // 最大迭代次数
        double convergenceThreshold = 1e-6;   // 收敛阈值
        bool enableUserWeights = false;       // 是否启用用户权重
    };

    /**
     * 切缝曲线表示
     */
    struct CutCurve {
        std::vector<Vector3> points;          // 曲线上的点
        std::vector<int> nearestFaces;        // 每个点最近的面
        double totalLength;                   // 曲线总长度
        double distortionReduction;           // 失真降低量
    };

    VariationalCutter() = default;
    ~VariationalCutter() = default;

    /**
     * 设置输入网格
     * @param mesh 输入的流形网格
     * @param geometry 对应的几何信息
     */
    void setMesh(std::shared_ptr<SurfaceMesh> mesh,
                 std::shared_ptr<VertexPositionGeometry> geometry);

    /**
     * 设置重要性/可见度权重
     * @param weights 每个顶点的权重（0-1之间，0表示完全隐藏，1表示完全可见）
     */
    void setImportanceWeights(const std::vector<double>& weights);

    /**
     * 执行变分切缝优化
     * @param params 切缝参数
     * @return 优化得到的切缝曲线集合
     */
    std::vector<CutCurve> computeOptimalCuts(const CuttingParams& params = CuttingParams{});

    /**
     * 将切缝应用到网格上
     * @param cuts 切缝曲线集合
     * @return 切开后的网格
     */
    std::shared_ptr<SurfaceMesh> applyCutsToMesh(const std::vector<CutCurve>& cuts);

    /**
     * 评估切缝质量
     */
    struct CutQuality {
        double totalLength;              // 总切缝长度
        double averageDistortion;        // 平均失真
        double maxDistortion;            // 最大失真
        int numCuts;                     // 切缝数量
        double visibilityScore;          // 可见性评分
    };

    CutQuality evaluateCutQuality(const std::vector<CutCurve>& cuts) const;

protected:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
    std::vector<double> importanceWeights_;

    /**
     * 内部网格表示（用于与EulerianShapeOptimizer集成）
     */
    void createInternalMeshRepresentation();

    /**
     * EulerianShapeOptimizer集成方法
     */
    std::optional<std::vector<CutCurve>> computeOptimalCutsWithESO(const CuttingParams& params);
    std::vector<CutCurve> computeOptimalCutsFallback(const CuttingParams& params);
    std::optional<std::vector<CutCurve>> extractCutsFromESO(class EulerianShapeOptimizer* optimizer);

    /**
     * 网格格式转换
     */
    // 移除有问题的声明，这些在实际实现中未使用
    // class HalfedgeMesh* convertFromGeometryCentral(std::shared_ptr<SurfaceMesh> gcMesh);
    // class Geometry<class Euclidean>* convertGeometry(
    //     std::shared_ptr<VertexPositionGeometry> gcGeometry,
    //     class HalfedgeMesh* mesh);

    /**
     * 计算Yamabe能量及其梯度
     */
    double computeYamabeEnergy() const;
    std::vector<Vector3> computeYamabeGradient() const;

    /**
     * 执行一步欧拉积分
     */
    void performEulerStep(std::vector<CutCurve>& curves,
                         double timeStep,
                         const CuttingParams& params);

    /**
     * 计算形状导数
     */
    std::vector<Vector3> computeShapeDerivative(const std::vector<CutCurve>& curves) const;

    /**
     * 检查收敛性
     */
    bool hasConverged(const std::vector<CutCurve>& oldCurves,
                     const std::vector<CutCurve>& newCurves,
                     double threshold) const;

    /**
     * 私有辅助方法
     */
    double computeEnergy(const std::vector<CutCurve>& cuts, const CuttingParams& params) const;
    std::vector<Vector3> computeGradient(const std::vector<CutCurve>& cuts, const CuttingParams& params) const;
    void updateCuts(std::vector<CutCurve>& cuts, const std::vector<Vector3>& gradient, double timeStep);
    double computeCurveLength(const CutCurve& cut) const;
    double computeDistortionEnergy(const std::vector<CutCurve>& cuts) const;
    double computeLengthEnergy(const std::vector<CutCurve>& cuts) const;
    double computeSmoothEnergy(const std::vector<CutCurve>& cuts) const;
    double computeAverageDistortion(const std::vector<CutCurve>& cuts) const;
    double computeMaxDistortion(const std::vector<CutCurve>& cuts) const;
    double computeVisibilityScore(const std::vector<CutCurve>& cuts) const;
};

} // namespace SurfaceTextureMapping