#pragma once

#include <vector>
#include <string>
#include <memory>
#include <optional>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "variational_cutting.h"

namespace SurfaceTextureMapping {

/**
 * 纹理映射类
 * 使用BFF(Boundary First Flattening)进行共形参数化
 */
class TextureMapper {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;
    using Vector2 = geometrycentral::Vector2;
    using Vector3 = geometrycentral::Vector3;

    struct MappingParams {
        bool useConformalMapping = true;      // 使用共形映射
        bool enableAreaCorrection = false;    // 启用面积校正
        double boundaryWeight = 1.0;          // 边界权重
        std::vector<int> coneVertices;        // 锥点顶点索引
        bool automaticConeDetection = true;   // 自动检测锥点
        double curvatureThreshold = 0.1;      // 曲率阈值用于锥点检测
    };

    /**
     * UV坐标映射结果
     */
    struct UVMapping {
        std::vector<Vector2> uvCoordinates;   // 每个顶点的UV坐标
        std::vector<std::vector<int>> charts; // 图块信息（每个图块包含的面索引）
        double totalDistortion;               // 总失真度
        double maxDistortion;                 // 最大失真度
        std::vector<double> faceDistortions;  // 每个面的失真度
    };

    TextureMapper() = default;
    ~TextureMapper() = default;

    /**
     * 设置输入网格（通常是已经切缝的网格）
     * @param mesh 输入网格
     * @param geometry 几何信息
     */
    void setMesh(std::shared_ptr<SurfaceMesh> mesh,
                 std::shared_ptr<VertexPositionGeometry> geometry);

    /**
     * 计算UV参数化
     * @param params 映射参数
     * @return UV映射结果
     */
    std::optional<UVMapping> computeUVMapping(const MappingParams& params = MappingParams{});

    /**
     * 自动检测锥点
     * @param curvatureThreshold 角缺陷阈值
     * @return 锥点顶点索引列表
     */
    std::vector<int> detectConeVertices(double curvatureThreshold = 0.1) const;

    /**
     * 优化UV布局
     * @param mapping 输入的UV映射
     * @param packingEfficiency 打包效率目标(0-1)
     * @return 优化后的UV映射
     */
    UVMapping optimizeUVLayout(const UVMapping& mapping,
                              double packingEfficiency = 0.8) const;

    /**
     * 导出带UV的OBJ文件
     * @param filename 输出文件名
     * @param mapping UV映射结果
     * @return 是否成功导出
     */
    bool exportUVMesh(const std::string& filename, const UVMapping& mapping) const;

    /**
     * 计算纹理失真度
     */
    struct DistortionMetrics {
        double angleDistortion;    // 角度失真
        double areaDistortion;     // 面积失真
        double conformalError;     // 共形误差
        std::vector<double> perFaceDistortion; // 每个面的失真
    };

    DistortionMetrics computeDistortionMetrics(const UVMapping& mapping) const;

    /**
     * 可视化UV映射
     * @param mapping UV映射结果
     * @param outputPath 输出图片路径
     * @param imageSize 图片尺寸
     */
    void visualizeUVMapping(const UVMapping& mapping,
                           const std::string& outputPath,
                           int imageSize = 1024) const;

protected:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;

    /**
     * BFF共形映射实现（真实算法集成）
     */
    std::optional<UVMapping> computeBFFMapping(const MappingParams& params);
    std::optional<UVMapping> computeBFFMappingIntegrated(const MappingParams& params);

    /**
     * 网格格式转换（用于BFF集成）
     */
    void createInternalMeshForBFF();
    // 移除有问题的声明，这些在实现中不需要使用
    // class HalfedgeMesh* convertMeshForBFF();
    // class Geometry<class Euclidean>* convertGeometryForBFF();

    /**
     * 计算边界约束
     */
    void setupBoundaryConstraints(const MappingParams& params);

    /**
     * Atlas打包算法
     */
    void packUVCharts(UVMapping& mapping, double efficiency) const;

    /**
     * 私有辅助方法
     */
    UVMapping computeConformalMapping(const MappingParams& params);
    UVMapping computeLSCMMapping(const MappingParams& params);
    void correctAreaDistortion(UVMapping& mapping);
    double computeTotalDistortion(const UVMapping& mapping) const;
    double computeMaximumDistortion(const UVMapping& mapping) const;
    double computeChartDistortion(size_t chartIndex, const UVMapping& mapping) const;
    double computeAngleDistortion(const UVMapping& mapping) const;
    double computeAreaDistortion(const UVMapping& mapping) const;
    double computeConformalError(const UVMapping& mapping) const;
};

} // namespace SurfaceTextureMapping