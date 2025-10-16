#pragma once

/**
 * 几何适配层 (Geometry Adapter Layer)
 * 在geometry-central和core几何库之间提供完整的双向转换
 *
 * 目标：
 * 1. 无损转换网格拓扑和几何
 * 2. 保持所有几何属性（长度、角度、面积等）
 * 3. 支持EulerianShapeOptimizer集成
 *
 * 版本: 1.0
 * 日期: 2025-10-09
 */

#include <memory>
#include <vector>
#include <map>
#include <unordered_map>

// geometry-central库 (新)
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

// 注意：不在头文件中包含Core库的头文件或前向声明
// 所有Core库类型仅作为不透明指针使用，完整定义在.cpp中

namespace SurfaceTextureMapping {

/**
 * 类型映射表
 * Core库类型使用void*表示，在.cpp中进行类型转换
 */
struct GeometryTypeMapping {
    // geometry-central --> core
    using GC_Mesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using GC_Geometry = geometrycentral::surface::VertexPositionGeometry;
    using GC_Vertex = geometrycentral::surface::Vertex;
    using GC_Edge = geometrycentral::surface::Edge;
    using GC_Face = geometrycentral::surface::Face;
    using GC_Halfedge = geometrycentral::surface::Halfedge;
    using GC_Vector3 = geometrycentral::Vector3;
    using GC_Vector2 = geometrycentral::Vector2;

    // core --> geometry-central (作为不透明指针)
    using Core_Mesh = void;
    using Core_Geometry = void;
};

/**
 * 几何适配器类
 * 提供geometry-central和core库之间的完整转换
 */
class GeometryAdapter {
public:
    /**
     * 转换选项
     */
    struct ConversionOptions {
        bool validateTopology = true;      // 验证拓扑一致性
        bool preserveAttributes = true;    // 保留所有几何属性
        bool verbose = false;              // 详细日志
    };

    /**
     * 转换结果
     */
    struct ConversionResult {
        bool success = false;
        std::string errorMessage;

        // 统计信息
        size_t numVertices = 0;
        size_t numEdges = 0;
        size_t numFaces = 0;

        // 几何误差
        double maxPositionError = 0.0;     // 顶点位置最大误差
        double maxEdgeLengthError = 0.0;   // 边长最大误差
        double maxAreaError = 0.0;         // 面积最大误差

        // 顶点索引映射
        std::vector<size_t> coreToGCVertexIndex;   // core顶点 -> GC索引
        std::vector<size_t> gcToCoreVertexIndex;   // GC顶点 -> core索引
    };

    /**
     * geometry-central --> core 转换
     *
     * @param gcMesh geometry-central网格
     * @param gcGeometry geometry-central几何
     * @param outMesh 输出：core网格 (调用者负责delete)
     * @param outGeometry 输出：core几何 (由outMesh管理，不需要单独delete)
     * @param options 转换选项
     * @return 转换结果
     */
    static ConversionResult convertFromGeometryCentral(
        std::shared_ptr<GeometryTypeMapping::GC_Mesh> gcMesh,
        std::shared_ptr<GeometryTypeMapping::GC_Geometry> gcGeometry,
        GeometryTypeMapping::Core_Mesh*& outMesh,
        GeometryTypeMapping::Core_Geometry*& outGeometry,
        const ConversionOptions& options = ConversionOptions{}
    );

    /**
     * core --> geometry-central 转换
     *
     * @param coreMesh core网格
     * @param coreGeometry core几何
     * @param outMesh 输出：geometry-central网格
     * @param outGeometry 输出：geometry-central几何
     * @param options 转换选项
     * @return 转换结果
     */
    static ConversionResult convertToGeometryCentral(
        const GeometryTypeMapping::Core_Mesh* coreMesh,
        const GeometryTypeMapping::Core_Geometry* coreGeometry,
        std::shared_ptr<GeometryTypeMapping::GC_Mesh>& outMesh,
        std::shared_ptr<GeometryTypeMapping::GC_Geometry>& outGeometry,
        const ConversionOptions& options = ConversionOptions{}
    );

    /**
     * 验证转换质量
     * 比较原始和转换后的网格，确保几何一致性
     */
    static ConversionResult validateConversion(
        std::shared_ptr<GeometryTypeMapping::GC_Mesh> originalMesh,
        std::shared_ptr<GeometryTypeMapping::GC_Geometry> originalGeometry,
        const GeometryTypeMapping::Core_Mesh* convertedMesh,
        const GeometryTypeMapping::Core_Geometry* convertedGeometry
    );

private:
    /**
     * 内部辅助方法
     */

    // 构建顶点映射表
    static void buildVertexMapping(
        std::shared_ptr<GeometryTypeMapping::GC_Mesh> gcMesh,
        const GeometryTypeMapping::Core_Mesh* coreMesh,
        std::unordered_map<size_t, size_t>& gc2core,
        std::unordered_map<size_t, size_t>& core2gc
    );

    // 验证拓扑等价
    static bool validateTopology(
        std::shared_ptr<GeometryTypeMapping::GC_Mesh> mesh1,
        const GeometryTypeMapping::Core_Mesh* mesh2
    );

    // 计算几何误差
    static void computeGeometricErrors(
        std::shared_ptr<GeometryTypeMapping::GC_Mesh> mesh1,
        std::shared_ptr<GeometryTypeMapping::GC_Geometry> geom1,
        const GeometryTypeMapping::Core_Geometry* geom2,
        ConversionResult& result
    );
};

/**
 * RAII包装器用于自动管理core网格生命周期
 */
class CoreMeshWrapper {
public:
    CoreMeshWrapper() : mesh_(nullptr), geometry_(nullptr) {}

    ~CoreMeshWrapper() {
        cleanup();
    }

    // 禁止拷贝
    CoreMeshWrapper(const CoreMeshWrapper&) = delete;
    CoreMeshWrapper& operator=(const CoreMeshWrapper&) = delete;

    // 支持移动
    CoreMeshWrapper(CoreMeshWrapper&& other) noexcept
        : mesh_(other.mesh_), geometry_(other.geometry_) {
        other.mesh_ = nullptr;
        other.geometry_ = nullptr;
    }

    GeometryTypeMapping::Core_Mesh* mesh() { return mesh_; }
    GeometryTypeMapping::Core_Geometry* geometry() { return geometry_; }

    void reset(GeometryTypeMapping::Core_Mesh* mesh,
               GeometryTypeMapping::Core_Geometry* geometry) {
        cleanup();
        mesh_ = mesh;
        geometry_ = geometry;
    }

    GeometryTypeMapping::Core_Mesh* releaseMesh() {
        auto temp = mesh_;
        mesh_ = nullptr;
        return temp;
    }

private:
    void cleanup() {
        if (mesh_) {
            delete mesh_;
            mesh_ = nullptr;
            // geometry_由mesh管理，不需要单独删除
            geometry_ = nullptr;
        }
    }

    GeometryTypeMapping::Core_Mesh* mesh_;
    GeometryTypeMapping::Core_Geometry* geometry_;
};

} // namespace SurfaceTextureMapping
