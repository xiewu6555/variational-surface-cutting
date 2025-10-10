#pragma once

#include <memory>
#include <vector>
#include <optional>
#include <string>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

#include "variational_cutting.h"
#include "texture_mapping.h"

// 前向声明内部网格类型 (来自cuts-core)
// 这些是来自variational-surface-cutting核心库的类型
class HalfedgeMesh;
template<typename T> class Geometry;
struct Vector3;

// 类型适配器命名空间，用于处理类型冲突
namespace Core {
    using EuclideanGeometry = Vector3;  // 适配内部的Euclidean定义
    using InternalMesh = HalfedgeMesh;
    using InternalGeometry = ::Geometry<EuclideanGeometry>;
}

namespace SurfaceTextureMapping {

/**
 * 真实算法集成接口
 * 提供EulerianShapeOptimizer和BFF算法的集成实现
 */
class RealAlgorithmIntegration {
public:
    using GCMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using GCGeometry = geometrycentral::surface::VertexPositionGeometry;

    /**
     * 创建集成的变分切割器
     * @return 使用真实EulerianShapeOptimizer的变分切割器
     */
    static std::unique_ptr<VariationalCutter> createIntegratedVariationalCutter();

    /**
     * 创建集成的纹理映射器
     * @return 使用真实BFF算法的纹理映射器
     */
    static std::unique_ptr<TextureMapper> createIntegratedTextureMapper();

    /**
     * 完整的处理流水线
     * @param mesh 输入网格
     * @param geometry 几何信息
     * @param cuttingParams 切割参数
     * @param mappingParams 映射参数
     * @param outputPath 输出路径
     * @return 是否成功处理
     */
    static bool processFullPipeline(
        std::shared_ptr<GCMesh> mesh,
        std::shared_ptr<GCGeometry> geometry,
        const VariationalCutter::CuttingParams& cuttingParams,
        const TextureMapper::MappingParams& mappingParams,
        const std::string& outputPath);

    /**
     * 测试集成功能
     * @param testMeshPath 测试网格路径
     * @return 测试是否通过
     */
    static bool runIntegrationTests(const std::string& testMeshPath);

    /**
     * 获取集成状态信息
     */
    struct IntegrationStatus {
        bool eulerianOptimizerAvailable = false;
        bool bffAvailable = false;
        bool meshConversionSupported = false;
        std::string statusMessage;
    };

    /**
     * 检查集成状态
     * @return 当前集成状态
     */
    static IntegrationStatus checkIntegrationStatus();
};

/**
 * 网格转换工具类
 * 负责在geometry-central和内部数据结构间转换
 */
class MeshConverter {
public:
    /**
     * 从geometry-central格式转换到内部格式
     */
    static bool convertFromGeometryCentral(
        std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh> gcMesh,
        std::shared_ptr<geometrycentral::surface::VertexPositionGeometry> gcGeometry,
        Core::InternalMesh*& outMesh,
        Core::InternalGeometry*& outGeometry);

    /**
     * 从内部格式转换到geometry-central格式
     */
    static bool convertToGeometryCentral(
        const Core::InternalMesh* inMesh,
        const Core::InternalGeometry* inGeometry,
        std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh>& outGCMesh,
        std::shared_ptr<geometrycentral::surface::VertexPositionGeometry>& outGCGeometry);

    /**
     * 验证网格转换的正确性
     */
    static bool validateConversion(
        std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh> originalMesh,
        const Core::InternalMesh* convertedMesh);
};

} // namespace SurfaceTextureMapping