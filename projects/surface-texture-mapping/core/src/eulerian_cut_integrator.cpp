/**
 * Eulerian Cut Integrator Implementation
 * 完整的Variational Cuts集成实现
 */

#include "eulerian_cut_integrator.h"
#include "boundary_extractor.h"
#include "geometry_adapter.h"
#include "eulerian_optimizer_wrapper.h"

// Core库（使用完整路径避免冲突）
#include "vector3.h"
#include "geometry.h"
#include "halfedge_mesh.h"
#include "eulerian_shape_optimizer.h"
#include "sdf_initialization.h"  // SDF初始化方法
#include "utilities.h"            // PI常量定义

#include <iostream>
#include <chrono>
#include <cmath>

namespace SurfaceTextureMapping {

/**
 * 主入口实现
 */
std::vector<std::vector<EulerianCutIntegrator::GC_Edge>>
EulerianCutIntegrator::generateCuts(
    GC_Mesh* mesh,
    GC_Geometry* geometry,
    const std::vector<GC_Vertex>& conePoints,
    const OptimizationParams& params,
    IntegrationResult& outResult) {

    std::vector<std::vector<GC_Edge>> result;
    outResult.success = false;

    auto startTime = std::chrono::high_resolution_clock::now();

    logProgress("=== Eulerian Cut Integration Started ===", params.verbose);

    try {
        // Step 1: 转换网格到core格式
        logProgress("Step 1: Converting mesh to core format...", params.verbose);

        GeometryTypeMapping::Core_Mesh* coreMesh = nullptr;
        GeometryTypeMapping::Core_Geometry* coreGeometry = nullptr;

        if (!convertMeshToCore(mesh, geometry, coreMesh, coreGeometry, outResult)) {
            throw std::runtime_error("Failed to convert mesh to core format");
        }

        // 使用RAII包装器管理core网格生命周期
        CoreMeshWrapper meshWrapper;
        meshWrapper.reset(coreMesh, coreGeometry);

        logProgress("  Conversion successful", params.verbose);
        logProgress("  Position error: " + std::to_string(outResult.conversionPositionError), params.verbose);

        // Step 2: 创建并配置优化器
        logProgress("Step 2: Creating Eulerian optimizer...", params.verbose);

        auto optimizer = createOptimizer(coreGeometry, params);
        if (!optimizer) {
            throw std::runtime_error("Failed to create optimizer");
        }

        logProgress("  Optimizer created successfully", params.verbose);

        // Step 3: 运行优化
        logProgress("Step 3: Running variational optimization...", params.verbose);

        runOptimization(*optimizer, params, outResult);

        logProgress("  Optimization completed: " + std::to_string(outResult.numIterations) + " iterations", params.verbose);
        logProgress("  Final energy: " + std::to_string(outResult.finalEnergy), params.verbose);

        // Step 4: 提取边界线
        logProgress("Step 4: Extracting boundary lines...", params.verbose);

        auto boundaries = extractBoundaryLines(*optimizer, params);

        logProgress("  Extracted " + std::to_string(boundaries.size()) + " boundary segments", params.verbose);

        // Step 5: 转换边界到边路径
        logProgress("Step 5: Converting boundaries to edge paths...", params.verbose);

        result = convertBoundariesToEdgePaths(
            boundaries, mesh, geometry,
            coreMesh, coreGeometry, params
        );

        logProgress("  Converted to " + std::to_string(result.size()) + " edge paths", params.verbose);

        // 计算统计信息
        computeStatistics(result, geometry, outResult);

        // 记录总耗时
        auto endTime = std::chrono::high_resolution_clock::now();
        outResult.computationTime = std::chrono::duration<double>(endTime - startTime).count();

        outResult.success = true;
        logProgress("=== Integration Completed Successfully ===", params.verbose);
        logProgress("Total time: " + std::to_string(outResult.computationTime) + " seconds", params.verbose);

    } catch (const std::exception& e) {
        outResult.errorMessage = std::string("Integration failed: ") + e.what();
        logError(outResult.errorMessage);
    }

    return result;
}

/**
 * Step 1: 转换网格
 */
bool EulerianCutIntegrator::convertMeshToCore(
    GC_Mesh* gcMesh,
    GC_Geometry* gcGeometry,
    GeometryTypeMapping::Core_Mesh*& outCoreMesh,
    GeometryTypeMapping::Core_Geometry*& outCoreGeometry,
    IntegrationResult& result) {

    std::shared_ptr<GC_Mesh> gcMeshShared(gcMesh, [](GC_Mesh*){});  // Non-owning shared_ptr
    std::shared_ptr<GC_Geometry> gcGeomShared(gcGeometry, [](GC_Geometry*){});

    GeometryAdapter::ConversionOptions options;
    options.validateTopology = true;
    options.preserveAttributes = true;
    options.verbose = true;  // 启用详细日志以调试

    auto conversionResult = GeometryAdapter::convertFromGeometryCentral(
        gcMeshShared, gcGeomShared,
        outCoreMesh, outCoreGeometry,
        options
    );

    if (!conversionResult.success) {
        result.errorMessage = "Mesh conversion failed: " + conversionResult.errorMessage;
        return false;
    }

    // 记录转换误差
    result.conversionPositionError = conversionResult.maxPositionError;
    result.conversionEdgeLengthError = conversionResult.maxEdgeLengthError;

    return true;
}

/**
 * Step 2: 创建优化器
 */
std::unique_ptr<EulerianShapeOptimizer>
EulerianCutIntegrator::createOptimizer(
    GeometryTypeMapping::Core_Geometry* coreGeometry,
    const OptimizationParams& params) {

    // 使用包装函数创建优化器
    // 这解决了typedef模板链接问题：包装函数在同一编译单元中完成类型转换和构造
    EulerianShapeOptimizer* rawOptimizer = EulerianOptimizerWrapper::createEulerianOptimizer(coreGeometry);

    // 包装为unique_ptr（标准deleter会调用delete，我们需要在wrapper的析构函数中调用destroyEulerianOptimizer）
    // 注意：由于wrapper只是调用delete，我们可以安全地使用标准deleter
    std::unique_ptr<EulerianShapeOptimizer> optimizer(rawOptimizer);

    // ========== 关键修复：设置初始状态 ==========
    std::cout << "  Initializing optimizer state with Normal Clustering..." << std::endl;

    // 使用Normal Clustering初始化（GitHub README推荐方法）
    // 显式类型转换：void* -> Geometry<Euclidean>*
    // 注意：Euclidean是Vector3的typedef，定义在geometry.h中
    // IMPORTANT: 使用reinterpret_cast而不是static_cast以避免类型检查问题
    Geometry<Euclidean>* geometry = reinterpret_cast<Geometry<Euclidean>*>(coreGeometry);

    // 调用normalClusterMSDF - 确保使用正确的函数签名
    VertexData<LabelVec> initialPhi = normalClusterMSDF(geometry, PI / 3.0);

    // 设置优化器状态（必须！）
    optimizer->setState(initialPhi);
    optimizer->iIter = 0;

    std::cout << "  State initialized successfully" << std::endl;
    // ========== 修复结束 ==========

    // 配置参数
    optimizer->weightLengthRegularization = params.weightLengthRegularization;
    optimizer->weightBilapRegularization = params.weightBilapRegularization;
    optimizer->weightHenckyDistortion = params.weightHenckyDistortion;
    optimizer->weightDirichletDistortion = params.weightDirichletDistortion;
    optimizer->weightVisibility = params.weightVisibility;
    optimizer->stepSizeParam = params.stepSizeParam;

    // 注意：initializeData()已经在setState()中调用，不需要再调用
    // 调用initializeData()会重新构建operators，这是我们之前观察到的"Building operators"卡住的地方

    return optimizer;
}

/**
 * Step 3: 运行优化
 */
void EulerianCutIntegrator::runOptimization(
    EulerianShapeOptimizer& optimizer,
    const OptimizationParams& params,
    IntegrationResult& result) {

    result.numIterations = 0;

    for (int i = 0; i < params.maxIterations; ++i) {
        std::cout << "[DEBUG] ===== Starting iteration " << i << " =====" << std::endl;

        // 执行一次梯度下降步骤
        std::cout << "[DEBUG] About to call doStep()..." << std::endl;
        std::cout.flush();

        try {
            optimizer.doStep();
            std::cout << "[DEBUG] doStep() completed successfully" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[DEBUG ERROR] doStep() threw exception: " << e.what() << std::endl;
            throw;
        } catch (...) {
            std::cerr << "[DEBUG ERROR] doStep() threw unknown exception" << std::endl;
            throw;
        }

        result.numIterations++;
        std::cout << "[DEBUG] Iteration count updated to " << result.numIterations << std::endl;

        // 每10次迭代输出进度
        if (params.verbose && (i + 1) % 10 == 0) {
            std::cout << "[DEBUG] About to compute energy for progress..." << std::endl;
            std::cout.flush();

            try {
                double energy = optimizer.computeEnergy(false);
                std::cout << "[DEBUG] Energy computed: " << energy << std::endl;
                logProgress("  Iteration " + std::to_string(i + 1) + "/" +
                           std::to_string(params.maxIterations) +
                           ", Energy: " + std::to_string(energy), true);
            } catch (const std::exception& e) {
                std::cerr << "[DEBUG ERROR] computeEnergy() threw exception: " << e.what() << std::endl;
                throw;
            } catch (...) {
                std::cerr << "[DEBUG ERROR] computeEnergy() threw unknown exception" << std::endl;
                throw;
            }
        }
    }

    // 计算最终能量
    std::cout << "[DEBUG] About to compute final energy..." << std::endl;
    std::cout.flush();

    try {
        result.finalEnergy = optimizer.computeEnergy(false);
        std::cout << "[DEBUG] Final energy computed: " << result.finalEnergy << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[DEBUG ERROR] Final computeEnergy() threw exception: " << e.what() << std::endl;
        throw;
    } catch (...) {
        std::cerr << "[DEBUG ERROR] Final computeEnergy() threw unknown exception" << std::endl;
        throw;
    }
}

/**
 * Step 4: 提取边界线
 */
std::vector<EulerianCutIntegrator::BoundaryLine>
EulerianCutIntegrator::extractBoundaryLines(
    EulerianShapeOptimizer& optimizer,  // 移除const
    const OptimizationParams& params) {

    std::vector<BoundaryLine> boundaries;

    // 从优化器获取边界线
    auto rawBoundaries = optimizer.getBoundaryLines();

    // 转换为BoundaryLine格式
    for (const auto& segment : rawBoundaries) {
        BoundaryLine boundary;
        boundary.start = GC_Vector3{segment[0].x, segment[0].y, segment[0].z};
        boundary.end = GC_Vector3{segment[1].x, segment[1].y, segment[1].z};
        boundary.segmentType = 0;  // 默认类型

        boundaries.push_back(boundary);
    }

    return boundaries;
}

/**
 * Step 5: 边界线转换为边路径
 */
std::vector<std::vector<EulerianCutIntegrator::GC_Edge>>
EulerianCutIntegrator::convertBoundariesToEdgePaths(
    const std::vector<BoundaryLine>& boundaries,
    GC_Mesh* gcMesh,
    GC_Geometry* gcGeometry,
    GeometryTypeMapping::Core_Mesh* coreMesh,
    GeometryTypeMapping::Core_Geometry* coreGeometry,
    const OptimizationParams& params) {

    // 转换BoundaryLine类型：EulerianCutIntegrator::BoundaryLine -> BoundaryExtractor::BoundaryLine
    std::vector<BoundaryExtractor::BoundaryLine> extractorBoundaries;
    extractorBoundaries.reserve(boundaries.size());

    for (const auto& boundary : boundaries) {
        BoundaryExtractor::BoundaryLine extractorBoundary;
        extractorBoundary.start = boundary.start;
        extractorBoundary.end = boundary.end;
        extractorBoundary.segmentType = boundary.segmentType;
        extractorBoundary.length = 0.0;  // 将在BoundaryExtractor中计算
        extractorBoundaries.push_back(extractorBoundary);
    }

    // 使用BoundaryExtractor进行转换
    BoundaryExtractor::ExtractionOptions options;
    options.snapTolerance = params.edgeSnapTolerance;
    options.useGeodesicPaths = params.useGeodesicPaths;
    options.verbose = params.verbose;

    BoundaryExtractor::ExtractionResult extractResult;

    // 显式类型转换：void* -> 具体Core类型
    auto* coreM = static_cast<BoundaryExtractor::Core_Mesh*>(coreMesh);
    auto* coreG = static_cast<BoundaryExtractor::Core_Geometry*>(coreGeometry);

    auto edgePaths = BoundaryExtractor::extractEdgePaths(
        extractorBoundaries, gcMesh, gcGeometry,
        coreM, coreG,
        options, extractResult
    );

    if (!extractResult.success) {
        logError("Boundary extraction failed: " + extractResult.errorMessage);
        // 返回空结果，调用者会回退到占位符
        return {};
    }

    if (params.verbose) {
        logProgress("  Extracted " + std::to_string(extractResult.numEdgesExtracted) + " edges", true);
        logProgress("  Average snap error: " + std::to_string(extractResult.averageSnapError), true);
    }

    return edgePaths;
}

/**
 * 计算统计信息
 */
void EulerianCutIntegrator::computeStatistics(
    const std::vector<std::vector<GC_Edge>>& cutPaths,
    GC_Geometry* geometry,
    IntegrationResult& result) {

    result.numCutPaths = cutPaths.size();
    result.totalCutEdges = 0;
    result.totalCutLength = 0.0;

    for (const auto& path : cutPaths) {
        result.totalCutEdges += path.size();

        for (const auto& edge : path) {
            result.totalCutLength += geometry->edgeLengths[edge];
        }
    }
}

/**
 * 日志辅助方法
 */
void EulerianCutIntegrator::logProgress(const std::string& message, bool verbose) {
    if (verbose) {
        std::cout << "[EulerianCutIntegrator] " << message << std::endl;
    }
}

void EulerianCutIntegrator::logError(const std::string& message) {
    std::cerr << "[EulerianCutIntegrator ERROR] " << message << std::endl;
}

} // namespace SurfaceTextureMapping
