#pragma once

#include <vector>
#include <string>
#include <memory>
#include <functional>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "texture_mapping.h"
#include "clipper2/clipper.h"

namespace SurfaceTextureMapping {

/**
 * 曲面填充类
 * 在UV空间生成各种图案和路径，然后映射回3D曲面
 */
class SurfaceFiller {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector2 = geometrycentral::Vector2;
    using Vector3 = geometrycentral::Vector3;
    using UVMapping = TextureMapper::UVMapping;

    /**
     * 填充图案类型
     */
    enum class PatternType {
        Grid,              // 网格
        Hexagonal,         // 六边形
        Concentric,        // 同心圆
        Spiral,            // 螺旋
        Hilbert,           // 希尔伯特曲线
        Peano,             // 皮亚诺曲线
        Custom             // 自定义图案
    };

    /**
     * 填充参数
     */
    struct FillingParams {
        PatternType type = PatternType::Grid;
        double spacing = 0.05;                     // 图案间距
        double lineWidth = 0.001;                  // 线宽
        bool respectBoundary = true;               // 是否遵循边界
        bool useFieldAlignment = false;           // 是否使用场对齐
        double fieldAlignmentWeight = 0.5;         // 场对齐权重
        int recursionDepth = 3;                    // 递归深度（用于分形曲线）
        std::function<Vector2(Vector2)> customPattern; // 自定义图案函数
    };

    /**
     * 填充路径结果
     */
    struct FillingResult {
        std::vector<std::vector<Vector3>> paths3D;  // 3D路径集合
        std::vector<std::vector<Vector2>> pathsUV;  // UV路径集合
        double totalLength;                         // 总长度
        double coverage;                            // 覆盖率
        int numPaths;                              // 路径数量
    };

    SurfaceFiller() = default;
    ~SurfaceFiller() = default;

    /**
     * 设置输入网格和UV映射
     * @param mesh 输入网格
     * @param geometry 几何信息
     * @param uvMapping UV映射结果
     */
    void setInput(std::shared_ptr<SurfaceMesh> mesh,
                  std::shared_ptr<VertexPositionGeometry> geometry,
                  const UVMapping& uvMapping);

    /**
     * 生成填充图案
     * @param params 填充参数
     * @return 填充结果
     */
    FillingResult generateFilling(const FillingParams& params) const;

    /**
     * 生成网格图案
     * @param spacing 网格间距
     * @param uvBounds UV边界
     * @return UV空间中的网格线段
     */
    std::vector<std::vector<Vector2>> generateGridPattern(
        double spacing, const std::vector<Vector2>& uvBounds) const;

    /**
     * 生成六边形图案
     * @param spacing 六边形大小
     * @param uvBounds UV边界
     * @return UV空间中的六边形图案
     */
    std::vector<std::vector<Vector2>> generateHexagonalPattern(
        double spacing, const std::vector<Vector2>& uvBounds) const;

    /**
     * 生成希尔伯特曲线
     * @param order 递归阶数
     * @param uvBounds UV边界
     * @return UV空间中的希尔伯特曲线
     */
    std::vector<Vector2> generateHilbertCurve(
        int order, const std::vector<Vector2>& uvBounds) const;

    /**
     * 生成皮亚诺曲线
     * @param order 递归阶数
     * @param uvBounds UV边界
     * @return UV空间中的皮亚诺曲线
     */
    std::vector<Vector2> generatePeanoCurve(
        int order, const std::vector<Vector2>& uvBounds) const;

    /**
     * 生成螺旋图案
     * @param spacing 螺旋间距
     * @param center 螺旋中心
     * @param maxRadius 最大半径
     * @return UV空间中的螺旋曲线
     */
    std::vector<Vector2> generateSpiralPattern(
        double spacing, const Vector2& center, double maxRadius) const;

    /**
     * 将UV路径映射回3D表面
     * @param uvPaths UV空间中的路径
     * @return 3D表面上的路径
     */
    std::vector<std::vector<Vector3>> mapPathsTo3D(
        const std::vector<std::vector<Vector2>>& uvPaths) const;

    /**
     * 使用Clipper2进行路径裁剪和布尔运算
     * @param paths 输入路径
     * @param clipPaths 裁剪路径
     * @param operation 布尔运算类型
     * @return 裁剪后的路径
     */
    std::vector<std::vector<Vector2>> clipPaths(
        const std::vector<std::vector<Vector2>>& paths,
        const std::vector<std::vector<Vector2>>& clipPaths,
        Clipper2Lib::ClipType operation = Clipper2Lib::ClipType::Intersection) const;

    /**
     * 导出路径为SVG文件（UV空间）
     * @param paths UV空间路径
     * @param filename 输出文件名
     * @param viewBox SVG视图框大小
     */
    void exportPathsToSVG(const std::vector<std::vector<Vector2>>& paths,
                         const std::string& filename,
                         double viewBox = 1.0) const;

    /**
     * 导出3D路径为OBJ线段
     * @param paths3D 3D路径
     * @param filename 输出文件名
     */
    void exportPaths3D(const std::vector<std::vector<Vector3>>& paths3D,
                       const std::string& filename) const;

    /**
     * 计算填充质量指标
     */
    struct FillingQuality {
        double uniformity;        // 均匀性
        double efficiency;        // 效率
        double boundaryRespect;   // 边界遵循度
        double smoothness;        // 平滑度
    };

    FillingQuality evaluateFillingQuality(const FillingResult& result) const;

private:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
    UVMapping uvMapping_;

    /**
     * 获取UV边界多边形
     */
    std::vector<std::vector<Vector2>> getUVBoundaries() const;

    /**
     * 检查点是否在UV域内
     */
    bool isPointInUVDomain(const Vector2& point) const;

    /**
     * 计算方向场（用于场对齐填充）
     */
    std::vector<Vector2> computeDirectionField() const;

    /**
     * 希尔伯特曲线递归生成
     */
    void hilbertRecursive(std::vector<Vector2>& curve,
                         Vector2 start, double size, int order, int direction) const;

    /**
     * 皮亚诺曲线递归生成
     */
    void peanoRecursive(std::vector<Vector2>& curve,
                       Vector2 start, double size, int order) const;

    /**
     * 辅助方法：计算路径总长度
     */
    double computeTotalLength(const std::vector<std::vector<Vector3>>& paths3D) const;

    /**
     * 辅助方法：计算覆盖率
     */
    double computeCoverage(const std::vector<std::vector<Vector2>>& pathsUV) const;

    /**
     * 辅助方法：计算间距变化
     */
    double computeSpacingVariation(const FillingResult& result) const;

    /**
     * 辅助方法：计算边界遵循度
     */
    double computeBoundaryRespect(const FillingResult& result) const;

    /**
     * 辅助方法：计算平滑度
     */
    double computeSmoothness(const FillingResult& result) const;
};

} // namespace SurfaceTextureMapping