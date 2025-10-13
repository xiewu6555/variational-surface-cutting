#pragma once

/**
 * Eulerian Shape Optimizer集成器
 * 协调geometry-central和EulerianShapeOptimizer之间的完整工作流
 *
 * 工作流程：
 * 1. 使用GeometryAdapter将GC网格转换为core格式
 * 2. 创建并配置EulerianShapeOptimizer
 * 3. 运行变分优化迭代
 * 4. 提取优化后的边界线
 * 5. 将边界线转换为GC网格的边路径
 *
 * 版本: 1.0
 * 日期: 2025-10-09
 */

#include <memory>
#include <vector>
#include <string>

// geometry-central类型
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

// 几何适配层
#include "geometry_adapter.h"

// Core库类型（前向声明）
class EulerianShapeOptimizer;

namespace SurfaceTextureMapping {

/**
 * Eulerian切割集成器
 */
class EulerianCutIntegrator {
public:
    // 类型别名
    using GC_Mesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using GC_Geometry = geometrycentral::surface::VertexPositionGeometry;
    using GC_Vertex = geometrycentral::surface::Vertex;
    using GC_Edge = geometrycentral::surface::Edge;
    using GC_Face = geometrycentral::surface::Face;
    using GC_Vector3 = geometrycentral::Vector3;

    /**
     * 优化参数
     */
    struct OptimizationParams {
        // Eulerian优化器参数
        double weightLengthRegularization = 1.0;    // 边界长度正则化
        double weightBilapRegularization = 1.0;     // Bilaplacian正则化
        double weightHenckyDistortion = 3.0;        // Hencky失真能量 (GitHub推荐值)
        double weightDirichletDistortion = 0.0;     // Dirichlet失真
        double weightVisibility = 0.0;              // 可见性能量

        // 迭代控制
        int maxIterations = 30;                     // 最大迭代次数（GitHub推荐：300，测试用30）
        double stepSizeParam = 0.01;                // 步长参数

        // 边界提取参数
        double edgeSnapTolerance = 1e-3;           // 边snap容差
        bool useGeodesicPaths = false;              // 使用测地线路径

        // 调试选项
        bool verbose = true;                        // 详细日志
        bool saveIntermediateResults = false;       // 保存中间结果
    };

    /**
     * 集成结果
     */
    struct IntegrationResult {
        bool success = false;
        std::string errorMessage;

        // 统计信息
        int numIterations = 0;                      // 实际迭代次数
        double finalEnergy = 0.0;                   // 最终能量
        double computationTime = 0.0;               // 计算时间（秒）

        // 几何转换误差
        double conversionPositionError = 0.0;       // 转换位置误差
        double conversionEdgeLengthError = 0.0;     // 转换边长误差

        // 切缝信息
        int numCutPaths = 0;                        // 切缝路径数量
        int totalCutEdges = 0;                      // 总切缝边数
        double totalCutLength = 0.0;                // 总切缝长度
    };

    EulerianCutIntegrator() = default;
    ~EulerianCutIntegrator() = default;

    /**
     * 主入口：生成切缝路径
     *
     * @param mesh geometry-central网格
     * @param geometry geometry-central几何
     * @param conePoints 锥点（高曲率点）
     * @param params 优化参数
     * @param outResult 输出结果信息
     * @return 切缝边路径列表
     */
    std::vector<std::vector<GC_Edge>> generateCuts(
        GC_Mesh* mesh,
        GC_Geometry* geometry,
        const std::vector<GC_Vertex>& conePoints,
        const OptimizationParams& params,
        IntegrationResult& outResult
    );

    /**
     * 简化接口（使用默认参数）
     */
    std::vector<std::vector<GC_Edge>> generateCuts(
        GC_Mesh* mesh,
        GC_Geometry* geometry,
        const std::vector<GC_Vertex>& conePoints
    ) {
        OptimizationParams params;
        IntegrationResult result;
        return generateCuts(mesh, geometry, conePoints, params, result);
    }

private:
    /**
     * 内部步骤方法
     */

    // Step 1: 转换网格到core格式
    bool convertMeshToCore(
        GC_Mesh* gcMesh,
        GC_Geometry* gcGeometry,
        GeometryTypeMapping::Core_Mesh*& outCoreMesh,
        GeometryTypeMapping::Core_Geometry*& outCoreGeometry,
        IntegrationResult& result
    );

    // Step 2: 创建并配置Eulerian优化器
    std::unique_ptr<EulerianShapeOptimizer> createOptimizer(
        GeometryTypeMapping::Core_Geometry* coreGeometry,
        const OptimizationParams& params
    );

    // Step 3: 运行优化迭代
    void runOptimization(
        EulerianShapeOptimizer& optimizer,
        const OptimizationParams& params,
        IntegrationResult& result
    );

    // Step 4: 提取边界线
    struct BoundarySegment {
        GC_Vector3 start;
        GC_Vector3 end;
        double length = 0.0;
    };

    struct BoundaryLine {
        GC_Vector3 start;
        GC_Vector3 end;
        int segmentType;  // BoundarySegment::BType的整数值（0=开放路径，1=闭环）
        double length = 0.0;  // 线段或路径长度
        std::vector<GC_Vector3> fullPath;  // 完整路径点序列（用于可视化）
        std::vector<BoundarySegment> segmentSequence;  // ⭐ 完整的线段序列（保留几何信息）
    };

    std::vector<BoundaryLine> extractBoundaryLines(
        EulerianShapeOptimizer& optimizer,  // 移除const，因为getBoundaryLines()不是const方法
        const OptimizationParams& params
    );

    // 辅助函数：合并连续的边界线段为路径
    std::vector<BoundaryLine> mergeBoundarySegments(
        const std::vector<BoundaryLine>& segments,
        double tolerance
    );

    // Step 5: 边界线转换为边路径
    std::vector<std::vector<GC_Edge>> convertBoundariesToEdgePaths(
        const std::vector<BoundaryLine>& boundaries,
        GC_Mesh* gcMesh,
        GC_Geometry* gcGeometry,
        GeometryTypeMapping::Core_Mesh* coreMesh,
        GeometryTypeMapping::Core_Geometry* coreGeometry,
        const OptimizationParams& params
    );

    // 辅助方法：构建顶点映射表
    void buildVertexMapping(
        GC_Mesh* gcMesh,
        GeometryTypeMapping::Core_Mesh* coreMesh,
        std::vector<size_t>& gc2core,
        std::vector<size_t>& core2gc
    );

    // 辅助方法：计算统计信息
    void computeStatistics(
        const std::vector<std::vector<GC_Edge>>& cutPaths,
        GC_Geometry* geometry,
        IntegrationResult& result
    );

    // 日志辅助
    void logProgress(const std::string& message, bool verbose);
    void logError(const std::string& message);
};

} // namespace SurfaceTextureMapping
