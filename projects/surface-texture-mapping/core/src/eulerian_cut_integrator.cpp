/**
 * Eulerian Cut Integrator Implementation
 * 完整的Variational Cuts集成实现
 * 真正的Variational Surface Cutting集成代码
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
#include <algorithm>
#include <unordered_map>

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

    (void)conePoints; // 当前集成版本尚未使用锥点信息

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
            coreMesh, coreGeometry,
            coreToGCVertexIndex_,
            params
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
    options.validateTopology = true;   // 启用拓扑验证
    options.preserveAttributes = false; // 禁用属性保留（因为geometry-central的MeshData访问有问题）
    options.verbose = true;            // 启用详细日志

    auto conversionResult = GeometryAdapter::convertFromGeometryCentral(
        gcMeshShared, gcGeomShared,
        outCoreMesh, outCoreGeometry,
        options
    );

    if (!conversionResult.success) {
        result.errorMessage = "Mesh conversion failed: " + conversionResult.errorMessage;
        return false;
    }

    coreToGCVertexIndex_ = conversionResult.coreToGCVertexIndex;
    gcToCoreVertexIndex_ = conversionResult.gcToCoreVertexIndex;

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

    // ========== 关键修复：在创建优化器之前先检查网格大小 ==========
    std::cout << "  [Pre-check] Validating mesh size before optimizer creation..." << std::endl;
    std::cout.flush();

    // 显式类型转换：void* -> Geometry<Euclidean>*
    Geometry<Euclidean>* geometry = reinterpret_cast<Geometry<Euclidean>*>(coreGeometry);

    // 验证geometry是否有效
    if (!geometry) {
        throw std::runtime_error("Geometry pointer is null after conversion");
    }

    HalfedgeMesh* mesh = geometry->getMesh();
    if (!mesh) {
        throw std::runtime_error("Mesh pointer is null in geometry");
    }

    int numVertices = mesh->nVertices();
    int numFaces = mesh->nFaces();
    int numEdges = mesh->nEdges();
    std::cout << "  Mesh stats: " << numVertices << " vertices, "
              << numFaces << " faces, " << numEdges << " edges" << std::endl;

    // 验证网格有效性
    if (numVertices == 0 || numFaces == 0) {
        throw std::runtime_error("Mesh has zero vertices or faces");
    }

    // ========== 性能检查：拒绝过大的网格 ==========
    const int PERFORMANCE_ERROR_THRESHOLD = 12000;  // 硬限制

    if (numVertices > PERFORMANCE_ERROR_THRESHOLD) {
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  ERROR: Mesh too dense for Variational Cutting!" << std::endl;
        std::cerr << "  Current vertices: " << numVertices << std::endl;
        std::cerr << "  Hard limit: " << PERFORMANCE_ERROR_THRESHOLD << " vertices" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  SOLUTION:" << std::endl;
        std::cerr << "    1. Enable 'Enable Remeshing' checkbox" << std::endl;
        std::cerr << "    2. Set 'Target Edge Length' to 0.035-0.045" << std::endl;
        std::cerr << "    3. Click 'Step 1: Process Mesh' FIRST" << std::endl;
        std::cerr << "    4. This will reduce vertex count to ~5k-8k" << std::endl;
        std::cerr << "    5. Then retry 'Step 2: Compute Cuts'" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  WHY: " << numVertices << " vertices would require " << std::endl;
        std::cerr << "       constructing sparse matrices with billions of entries," << std::endl;
        std::cerr << "       leading to memory exhaustion or hours of computation." << std::endl;
        std::cerr << "  ========================================" << std::endl;

        throw std::runtime_error("Mesh too dense: " + std::to_string(numVertices) +
                               " vertices exceeds hard limit of " + std::to_string(PERFORMANCE_ERROR_THRESHOLD));
    }

    std::cout << "  ✓ Mesh size check passed" << std::endl;

    // ========== 关键验证：检查网格质量（边长均匀性） ==========
    // Variational Surface Cutting算法对网格质量有严格要求
    std::cout << "  Checking mesh quality for numerical stability..." << std::endl;

    double minEdgeLength = 1e10;
    double maxEdgeLength = 0.0;
    double totalEdgeLength = 0.0;

    // Core库API：使用边ID遍历（0到nEdges-1）
    for (int edgeId = 0; edgeId < numEdges; ++edgeId) {
        EdgePtr e = mesh->edge(edgeId);
        double len = geometry->length(e);
        minEdgeLength = std::min(minEdgeLength, len);
        maxEdgeLength = std::max(maxEdgeLength, len);
        totalEdgeLength += len;
    }

    double avgEdgeLength = totalEdgeLength / numEdges;
    double edgeLengthRatio = maxEdgeLength / minEdgeLength;

    std::cout << "    Min edge length: " << minEdgeLength << std::endl;
    std::cout << "    Max edge length: " << maxEdgeLength << std::endl;
    std::cout << "    Avg edge length: " << avgEdgeLength << std::endl;
    std::cout << "    Edge length ratio (max/min): " << edgeLengthRatio << std::endl;

    // 如果边长差异过大（ratio > 10），拒绝运行
    const double MAX_ACCEPTABLE_RATIO = 10.0;
    if (edgeLengthRatio > MAX_ACCEPTABLE_RATIO) {
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  ERROR: Mesh quality is insufficient!" << std::endl;
        std::cerr << "  Edge length ratio (" << edgeLengthRatio << ") exceeds threshold (" << MAX_ACCEPTABLE_RATIO << ")" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  SOLUTION:" << std::endl;
        std::cerr << "    1. In the GUI, execute 'Step 1: Process Mesh (Remesh)' FIRST" << std::endl;
        std::cerr << "    2. Set 'Target Edge Length' to 0.005 or smaller" << std::endl;
        std::cerr << "    3. Enable 'Enable Remeshing' checkbox" << std::endl;
        std::cerr << "    4. Then execute 'Step 2: Compute Cuts'" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  WHY: Variational Surface Cutting requires uniform mesh for numerical stability." << std::endl;
        std::cerr << "       Non-uniform edges cause matrix ill-conditioning and factorization failure." << std::endl;
        std::cerr << "  ========================================" << std::endl;

        throw std::runtime_error("Mesh quality check failed: edge length ratio too large. Please remesh first!");
    }

    // ⭐⭐⭐ 检查最小网格分辨率（避免"no interior vertices"错误）
    //
    // 基于ANALYSIS_no_interior_vertices_deep_dive.md的深度分析结果：
    // - Normal Clustering在低分辨率网格上会产生**拓扑退化的patches**
    // - 退化patches的所有顶点都在边界上（nInterior=0），无法求解Yamabe方程
    // - 即使总顶点数达标，patch分布可能极不均匀（尖端区域会产生退化patches）
    //
    // 实测数据（2025-10-13深度分析）：
    // - 780顶点：Patch 5退化（nInterior=0），失败 ❌
    // - 1200顶点：仍然可能有退化patches，不够可靠 ⚠️
    // - 1400顶点（r=0.0075）：退化patches减少，基本可用 ✅
    // - 3000顶点（r=0.005）：完全消除退化patches，稳定成功 ✅✅✅
    //
    // 设计原则：
    // 1. 每个patch需要至少50个内部顶点（Yamabe求解器的数值稳定性要求）
    // 2. 考虑不均匀分布：尖端区域可能只分到20%的平均顶点数
    // 3. 安全系数：实际需求 × 5（应对最坏情况的分布不均）
    //
    // 计算逻辑：
    // - MIN_INTERIOR_PER_PATCH = 50（内部顶点）
    // - 假设内部顶点比例 = 60%（边界顶点占40%）
    // - MIN_TOTAL_PER_PATCH = 50 / 0.6 ≈ 83
    // - 考虑不均匀分布安全系数 × 5 = 83 × 5 ≈ 415
    // - 实际采用保守值：MIN_VERTICES_PER_PATCH = 500（确保鲁棒性）
    // - EXPECTED_NUM_PATCHES = 6-10（根据Normal Clustering的典型输出）
    // - 理想MIN_TOTAL_VERTICES = 500 × 6 = 3000
    // - 实际采用：5000（考虑最坏情况10个patches + 额外安全边际）
    const int MIN_VERTICES_PER_PATCH = 500;  // 大幅提升from 200（基于深度分析）
    const int EXPECTED_NUM_PATCHES = 10;     // 提升from 6（保守估计）
    const int MIN_TOTAL_VERTICES = 5000;     // 硬编码明确值（= 500 × 10）

    if (numVertices < MIN_TOTAL_VERTICES) {
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  ERROR: Mesh resolution too low!" << std::endl;
        std::cerr << "  Current vertices: " << numVertices << " (minimum required: " << MIN_TOTAL_VERTICES << ")" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  CRITICAL: This will cause 'no interior vertices' error!" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  SOLUTION:" << std::endl;
        std::cerr << "    1. Enable 'Enable Remeshing' checkbox in GUI" << std::endl;
        std::cerr << "    2. Set 'Target Edge Length' based on mesh size:" << std::endl;
        std::cerr << "       - For small models (diagonal < 2.0): use 0.008-0.012" << std::endl;
        std::cerr << "       - For medium models (diagonal 2-5): use 0.015-0.020" << std::endl;
        std::cerr << "       - For large models (diagonal > 5): use 0.025-0.030" << std::endl;
        std::cerr << "    3. Set 'Remesh Iterations' to 10-15 (higher = better quality)" << std::endl;
        std::cerr << "    4. Click 'Step 1: Process Mesh' and wait for completion" << std::endl;
        std::cerr << "    5. Verify vertex count is 5k-8k in Statistics panel" << std::endl;
        std::cerr << "       (If > 8k, increase Target Edge Length to avoid 12k limit)" << std::endl;
        std::cerr << "    6. Then retry 'Step 2: Compute Cuts'" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  WHY: Normal Clustering creates patches with varying sizes." << std::endl;
        std::cerr << "       Small patches (e.g., at sharp corners) may have ALL vertices" << std::endl;
        std::cerr << "       on the boundary (nInterior=0), causing Yamabe solver to fail." << std::endl;
        std::cerr << "       Need " << MIN_TOTAL_VERTICES << "+ total vertices to ensure EVERY patch" << std::endl;
        std::cerr << "       has at least 50 interior vertices for numerical stability." << std::endl;
        std::cerr << "       Current: " << numVertices << " vertices (deficit: "
                  << (MIN_TOTAL_VERTICES - numVertices) << ")" << std::endl;
        std::cerr << "  ========================================" << std::endl;
        std::cerr << "  REFERENCE: See ANALYSIS_no_interior_vertices_deep_dive.md" << std::endl;
        std::cerr << "  ========================================" << std::endl;

        throw std::runtime_error("Mesh resolution too low. Minimum " + std::to_string(MIN_TOTAL_VERTICES) +
                                " vertices required, but only " + std::to_string(numVertices) + " found.");
    }

    // ⭐ 提供边长建议（基于顶点数）
    double recommendedAvgEdgeLength = 0.01;  // 默认推荐值
    if (numVertices > 10000) {
        recommendedAvgEdgeLength = 0.03;  // 大网格使用较大边长
    } else if (numVertices > 5000) {
        recommendedAvgEdgeLength = 0.02;
    }

    if (avgEdgeLength < recommendedAvgEdgeLength * 0.5) {
        std::cout << "  ⚠ Warning: Average edge length (" << avgEdgeLength << ") might be too small." << std::endl;
        std::cout << "            For " << numVertices << " vertices, recommended avg edge length is ~"
                  << recommendedAvgEdgeLength << std::endl;
    }

    std::cout << "  ✓ Mesh quality check passed" << std::endl;

    // 检查网格是否是流形（HalfedgeMesh应该保证这一点）
    std::cout << "  Mesh is manifold: Yes (guaranteed by HalfedgeMesh)" << std::endl;

    // 检查网格拓扑（Core库使用nBoundaryLoops()而不是hasBoundary()）
    size_t numBoundaryLoops = mesh->nBoundaryLoops();
    std::cout << "  Mesh has boundary: " << (numBoundaryLoops > 0 ? "Yes" : "No") << std::endl;
    std::cout << "  Number of boundary loops: " << numBoundaryLoops << std::endl;
    if (numBoundaryLoops > 0) {
        std::cout << "  Warning: Mesh has boundary. Algorithm may need boundary conditions." << std::endl;
    }

    // ========== 在所有检查通过后，创建优化器 ==========
    std::cout << "  Creating EulerianShapeOptimizer..." << std::endl;
    std::cout.flush();

    std::unique_ptr<EulerianShapeOptimizer> optimizer;
    try {
        // 使用wrapper函数创建优化器（解决跨编译单元模板链接问题）
        EulerianShapeOptimizer* rawOptimizer = EulerianOptimizerWrapper::createEulerianOptimizer(coreGeometry);
        optimizer.reset(rawOptimizer);
        std::cout << "  ✓ Optimizer created successfully" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ERROR: Failed to create optimizer: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Optimizer creation failed: ") + e.what());
    }

    // 设置优化器参数（在setState之前）
    std::cout << "  Configuring optimizer parameters..." << std::endl;
    optimizer->weightLengthRegularization = params.weightLengthRegularization;
    optimizer->weightHenckyDistortion = params.weightHenckyDistortion;
    optimizer->weightDirichletDistortion = params.weightDirichletDistortion;
    optimizer->weightBilapRegularization = params.weightBilapRegularization;
    optimizer->weightVisibility = params.weightVisibility;
    optimizer->stepSizeParam = params.stepSizeParam;
    // 关键：启用局部尺度长度正则，使边界有非零梯度（不依赖贴片内部点）
    optimizer->localScaleLengthRegularization = true;
    std::cout << "    Length regularization weight: " << optimizer->weightLengthRegularization << std::endl;
    std::cout << "    Hencky distortion weight: " << optimizer->weightHenckyDistortion << std::endl;
    std::cout << "    Bilap regularization weight: " << optimizer->weightBilapRegularization << std::endl;
    std::cout << "    Dirichlet distortion weight: " << optimizer->weightDirichletDistortion << std::endl;
    std::cout << "    Visibility weight: " << optimizer->weightVisibility << std::endl;
    std::cout << "    Step size param: " << optimizer->stepSizeParam << std::endl;

    // ========== 初始化优化器状态（使用normal clustering） ==========
    // 调用normalClusterMSDF - 确保使用正确的函数签名
    std::cout << "  Calling normalClusterMSDF()..." << std::endl;
    std::cout.flush();

    VertexData<LabelVec> initialPhi;
    try {
        initialPhi = normalClusterMSDF(geometry, PI / 3.0);
        std::cout << "  Normal clustering completed successfully" << std::endl;

        // 注意：Core库的VertexData没有raw()方法来检查大小
        // 我们信任normalClusterMSDF的返回值 - 如果它成功返回，数据应该是有效的
        std::cout << "  Initial phi data created for " << numVertices << " vertices" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ERROR: normalClusterMSDF failed: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Normal clustering initialization failed: ") + e.what());
    }

    // 注意：normalClusterMSDF已经被验证在原始项目中正常工作
    // 直接信任其结果，避免复杂的Core API遍历

    std::cout << "  Setting optimizer state..." << std::endl;
    std::cout.flush();

    // 设置优化器状态（必须！）
    try {
        std::cout << "  [DEBUG] About to call setState()..." << std::endl;
        std::cout.flush();

        optimizer->setState(initialPhi);

        std::cout << "  [DEBUG] setState() returned successfully" << std::endl;
        std::cout.flush();

        std::cout << "  State initialized successfully" << std::endl;
        std::cout << "  Note: setState() has called initializeData() to build operators" << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ERROR: setState() failed: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Optimizer state initialization failed: ") + e.what());
    } catch (...) {
        std::cerr << "  ERROR: setState() threw unknown exception" << std::endl;
        throw std::runtime_error("Optimizer state initialization failed with unknown exception");
    }
    // ========== 修复结束 ==========

    std::cout << "  [DEBUG] About to return optimizer..." << std::endl;
    std::cout.flush();

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

    // 在第一次迭代前，验证优化器状态
    std::cout << "  Verifying optimizer state before first iteration..." << std::endl;
    std::cout << "    iIter = " << optimizer.iIter << std::endl;
    std::cout << "    weightLengthRegularization = " << optimizer.weightLengthRegularization << std::endl;
    std::cout << "    weightHenckyDistortion = " << optimizer.weightHenckyDistortion << std::endl;
    std::cout.flush();

    // 尝试计算初始能量（这会触发矩阵构建）
    std::cout << "  Computing initial energy (this will build operators)..." << std::endl;
    std::cout.flush();

    try {
        double initialEnergy = optimizer.computeEnergy(false);
        std::cout << "  Initial energy: " << initialEnergy << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "  ERROR: Failed to compute initial energy: " << e.what() << std::endl;
        std::cerr << "  This suggests a problem with matrix construction or initial state" << std::endl;
        throw std::runtime_error(std::string("Initial energy computation failed: ") + e.what());
    }

    std::cout << "  Starting optimization iterations..." << std::endl;

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
            std::cerr << "[DEBUG ERROR] Failed at iteration " << i << std::endl;
            // 自愈重试：首轮失败时，减小步长并重试，最多3次（仍为Eulerian算法）
            bool recovered = false;
            double origStep = optimizer.stepSizeParam;
            for (int retry = 0; retry < 3 && i == 0 && !recovered; ++retry) {
                optimizer.stepSizeParam *= 0.5; // 更保守的步长
                std::cerr << "[RETRY] Reducing stepSizeParam to " << optimizer.stepSizeParam << " and retrying doStep()..." << std::endl;
                try {
                    optimizer.doStep();
                    recovered = true;
                    std::cerr << "[RETRY] doStep() succeeded after step reduction" << std::endl;
                } catch (...) {
                    // continue
                }
            }
            if (!recovered) {
                // 恢复原步长并抛出
                optimizer.stepSizeParam = origStep;
                throw;
            }
        } catch (...) {
            std::cerr << "[DEBUG ERROR] doStep() threw unknown exception" << std::endl;
            std::cerr << "[DEBUG ERROR] Failed at iteration " << i << std::endl;
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
* 切割线几何侧问题（根因）
  - 边界段逐段独立贴边，未先拼成连续折线
      - eulerian_cut_integrator.cpp::extractBoundaryLines()只是把优化器返回的边界线段 optimizer.getBoundaryLines() 逐段
  塞进 BoundaryLine，没有按端点连通把“同一条边界的所有小段”串成一条有序折线，BoundaryLine::segmentSequence 也没有被真正
  填充/使用。
      - 随后 convertBoundariesToEdgePaths() 里按段调用 BoundaryExtractor::extractEdgePaths()：每段只把两个端点各自“snap
  到最近网格顶点”，再跑一次 Dijkstra 找“最短边路径”。段与段之间既无顺序约束，也不保证端点对齐同一顶点，最后只是简单拼接/
  串联（见“合并”部分仅做了 insert，没有按端点重排/翻转保证连通）。结果就是每一小段都朝自己最近的边网走，整体就断裂、偏离
  目标曲线。
  - 边路径合并未保证顺序与连通性
      - convertBoundariesToEdgePaths() 末尾“合并 edge paths 回到 boundary”直接把多段 path 连接到 mergedPath，没有按“上段
  尾顶点 == 下段首顶点”重排/必要时反转，导致最终路径顺序错乱、断裂。
  - 端点“就近贴顶点”对窄特征不鲁棒
      - boundary_extractor.cpp::findNearestVertex() 采用全局最近点，未限制到“靠近原边界段的邻域”，在特征密集处常把端点吸
  到错误一侧，之后 Dijkstra 就会横跨错误面群，出现你图中“离开表面正确边界”的折返/游离。
 */
std::vector<EulerianCutIntegrator::BoundaryLine>
EulerianCutIntegrator::extractBoundaryLines(
    EulerianShapeOptimizer& optimizer,  // 移除const
    const OptimizationParams& params) {

    optimizerBoundarySegments_.clear();
    const auto& rawSegmentsFromOptimizer = optimizer.getBoundarySegments();
    optimizerBoundarySegments_ = rawSegmentsFromOptimizer; // 拷贝一份，保持生命周期

    size_t totalSegments = 0;
    for (const auto& regionSegments : optimizerBoundarySegments_) {
        totalSegments += regionSegments.size();
    }

    std::vector<BoundarySegmentInfo> rawSegments;
    rawSegments.reserve(totalSegments);

    for (const auto& regionSegments : optimizerBoundarySegments_) {
        for (const auto& seg : regionSegments) {
            BoundarySegmentInfo info;
            Vector3 start = optimizer.startPoint(seg);
            Vector3 end = optimizer.endPoint(seg);

            info.start = GC_Vector3{start.x, start.y, start.z};
            info.end = GC_Vector3{end.x, end.y, end.z};
            info.length = optimizer.boundaryLength(seg);
            info.segmentType = static_cast<int>(seg.type);
            if (seg.type == BType::TRIPLE) {
                info.intermediatePoints.push_back(
                    GC_Vector3{seg.triplePoint.x, seg.triplePoint.y, seg.triplePoint.z}
                );
            }
            info.sourceSegment = &seg;

            rawSegments.push_back(std::move(info));
        }
    }

    std::cout << "  Raw boundary segments from optimizer: " << rawSegments.size() << std::endl;

    // ========== 关键修复：合并连续的线段为路径 ==========
    std::cout << "  Merging boundary segments into continuous paths..." << std::endl;

    double mergeTol = std::max(1e-6, params.edgeSnapTolerance);
    std::vector<BoundaryLine> mergedBoundaries = mergeBoundarySegments(rawSegments, mergeTol);

    std::cout << "  After merging: " << mergedBoundaries.size() << " boundary paths" << std::endl;

    // 统计路径长度
    if (!mergedBoundaries.empty()) {
        double totalLength = 0.0;
        for (const auto& b : mergedBoundaries) {
            totalLength += b.length;
        }
        std::cout << "  Average path length: " << (totalLength / mergedBoundaries.size()) << std::endl;
    }

    return mergedBoundaries;
}

/**
 * 辅助函数：合并连续的边界线段为路径
 *
 * ⭐ 修复后的策略：不丢弃中间几何信息，保留所有线段并分组
 *
 * 关键修复：原来的实现只保存了端点，导致生成直线而不是曲面路径
 * 现在：保留完整的线段序列，只是对它们进行分组和排序
 */
std::vector<EulerianCutIntegrator::BoundaryLine>
EulerianCutIntegrator::mergeBoundarySegments(
    const std::vector<BoundarySegmentInfo>& segments,
    double tolerance) {

    if (segments.empty()) return {};

    // 标记已使用的线段
    std::vector<bool> used(segments.size(), false);
    std::vector<BoundaryLine> mergedPaths;

    auto pointDistance = [](const GC_Vector3& a, const GC_Vector3& b) {
        double dx = a.x - b.x;
        double dy = a.y - b.y;
        double dz = a.z - b.z;
        return std::sqrt(dx*dx + dy*dy + dz*dz);
    };

    auto reverseSegment = [](const BoundarySegmentInfo& segment) {
        BoundarySegmentInfo reversed = segment;
        std::swap(reversed.start, reversed.end);
        std::reverse(reversed.intermediatePoints.begin(), reversed.intermediatePoints.end());
        reversed.sourceSegment = segment.sourceSegment;
        reversed.reversed = !segment.reversed;
        return reversed;
    };

    // 贪心分组：从每个未使用的线段开始，找到连续的线段序列
    for (size_t i = 0; i < segments.size(); ++i) {
        if (used[i]) continue;

        // 开始新路径 - 保存完整的线段序列，而不只是端点
        std::vector<BoundarySegmentInfo> pathSegments;
        pathSegments.push_back(segments[i]);
        used[i] = true;

        GC_Vector3 currentEnd = segments[i].end;
        bool extended = true;

        // 向前扩展路径（连接currentEnd）
        while (extended) {
            extended = false;
            double bestDist = tolerance;
            int bestIdx = -1;
            bool needReverse = false;  // 是否需要反转找到的线段

            // 查找最近的未使用线段
            for (size_t j = 0; j < segments.size(); ++j) {
                if (used[j]) continue;
                const auto& candidate = segments[j];

                double distToStart = pointDistance(currentEnd, candidate.start);
                double distToEnd = pointDistance(currentEnd, candidate.end);

                if (distToStart < bestDist) {
                    bestDist = distToStart;
                    bestIdx = j;
                    needReverse = false;  // 正向添加
                } else if (distToEnd < bestDist) {
                    bestDist = distToEnd;
                    bestIdx = j;
                    needReverse = true;  // 需要反转
                }
            }

            if (bestIdx >= 0) {
                // 添加找到的线段到路径
                BoundarySegmentInfo segmentToAdd;
                if (needReverse) {
                    // 反转线段方向
                    segmentToAdd = reverseSegment(segments[bestIdx]);
                } else {
                    segmentToAdd = segments[bestIdx];
                }

                pathSegments.push_back(segmentToAdd);
                currentEnd = segmentToAdd.end;
                used[bestIdx] = true;
                extended = true;
            }
        }

        // 检查路径是否形成闭环
        double loopDist = pointDistance(pathSegments.front().start, pathSegments.back().end);
        bool isLoop = (loopDist < tolerance);

        // 计算路径总长度并构建点序列（用于调试和可视化）
        double totalLength = 0.0;
        std::vector<GC_Vector3> pathPoints;
        pathPoints.push_back(pathSegments[0].start);

        for (const auto& seg : pathSegments) {
            for (const auto& mid : seg.intermediatePoints) {
                pathPoints.push_back(mid);
            }
            pathPoints.push_back(seg.end);
            totalLength += seg.length;
        }

        // 如果是闭环，闭合路径
        if (isLoop && pathPoints.size() > 2) {
            pathPoints.push_back(pathPoints.front());
        }

        // 创建合并后的BoundaryLine，存储完整的线段序列
        BoundaryLine mergedPath;
        mergedPath.start = pathSegments.front().start;
        mergedPath.end = pathSegments.back().end;
        mergedPath.length = totalLength;
        mergedPath.segmentType = isLoop ? 1 : 0;
        mergedPath.fullPath = pathPoints;  // ⭐ 用于可视化调试

        // ⭐⭐ 关键：存储完整的线段序列，而不只是端点
        // 后续convertBoundariesToEdgePaths会使用这些线段
        mergedPath.segmentSequence = pathSegments;

        mergedPaths.push_back(mergedPath);

        std::cout << "  Merged path: " << pathSegments.size() << " segments, "
                 << "length=" << totalLength
                 << (isLoop ? " (closed loop)" : " (open path)") << std::endl;
    }

    return mergedPaths;
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

    return convertBoundariesToEdgePaths(
        boundaries, gcMesh, gcGeometry,
        coreMesh, coreGeometry,
        coreToGCVertexIndex_,
        params
    );
}

std::vector<std::vector<EulerianCutIntegrator::GC_Edge>>
EulerianCutIntegrator::convertBoundariesToEdgePaths(
    const std::vector<BoundaryLine>& boundaries,
    GC_Mesh* gcMesh,
    GC_Geometry* gcGeometry,
    GeometryTypeMapping::Core_Mesh* coreMesh,
    GeometryTypeMapping::Core_Geometry* coreGeometry,
    const std::vector<size_t>& coreToGCVertexIndex,
    const OptimizationParams& params) {

    (void)gcGeometry;
    (void)coreGeometry;

    if (coreToGCVertexIndex.empty()) {
        logError("Vertex mapping not initialized; aborting edge path conversion.");
        return {};
    }

    auto* coreM = static_cast<BoundaryExtractor::Core_Mesh*>(coreMesh);

    if (coreM == nullptr) {
        logError("Core mesh pointer is null; cannot convert boundaries.");
        return {};
    }

    auto collectCoreEdges = [&](const BoundarySegmentInfo& segInfo) {
        std::vector<EdgePtr> edges;
        if (!segInfo.sourceSegment) {
            return edges;
        }

        const ::BoundarySegment& source = *segInfo.sourceSegment;
        HalfedgePtr current = segInfo.reversed ? source.heEnd : source.heStart;
        HalfedgePtr target = segInfo.reversed ? source.heStart : source.heEnd;

        if (current == nullptr) {
            return edges;
        }

        const size_t guardLimit = coreM->nHalfedges() + 1;
        size_t guard = 0;

        auto appendEdge = [&](EdgePtr e) {
            if (e == nullptr) return;
            if (edges.empty() || edges.back() != e) {
                edges.push_back(e);
            }
        };

        appendEdge(current.edge());

        while (!(current == target) && guard < guardLimit) {
            current = segInfo.reversed ? current.prev() : current.next();
            if (current == nullptr) {
                break;
            }
            appendEdge(current.edge());
            ++guard;
        }

        if (!(current == target)) {
            std::cerr << "[EulerianCutIntegrator] Warning: Failed to trace boundary segment along halfedges; segment may be degenerate." << std::endl;
        }

        return edges;
    };

    std::vector<std::vector<GC_Edge>> gcEdgePaths;
    gcEdgePaths.reserve(boundaries.size());

    size_t boundaryIdx = 0;
    for (const auto& boundary : boundaries) {
        std::vector<EdgePtr> coreEdges;
        for (const auto& seg : boundary.segmentSequence) {
            auto segmentEdges = collectCoreEdges(seg);
            if (segmentEdges.empty()) {
                continue;
            }

            if (!coreEdges.empty() && !segmentEdges.empty() && coreEdges.back() == segmentEdges.front()) {
                segmentEdges.erase(segmentEdges.begin());
            }
            coreEdges.insert(coreEdges.end(), segmentEdges.begin(), segmentEdges.end());
        }

        if (coreEdges.empty()) {
            if (params.verbose) {
                std::cerr << "[EulerianCutIntegrator] Warning: Boundary " << boundaryIdx << " produced no core edges." << std::endl;
            }
            ++boundaryIdx;
            continue;
        }

        // 去除连续重复的边，保持路径紧凑
        std::vector<EdgePtr> deduped;
        deduped.reserve(coreEdges.size());
        for (const auto& e : coreEdges) {
            if (deduped.empty() || deduped.back() != e) {
                deduped.push_back(e);
            }
        }

        auto gcPath = BoundaryExtractor::mapCoreEdgesToGC(
            deduped,
            coreM,
            gcMesh,
            coreToGCVertexIndex
        );

        if (gcPath.empty()) {
            std::cerr << "[EulerianCutIntegrator] Warning: Failed to map core edges to GC edges for boundary "
                      << boundaryIdx << std::endl;
            ++boundaryIdx;
            continue;
        }

        if (!BoundaryExtractor::validateEdgePath(gcPath, gcMesh)) {
            std::cerr << "[EulerianCutIntegrator] Warning: GC edge path not contiguous for boundary "
                      << boundaryIdx << std::endl;
        }

        gcEdgePaths.push_back(std::move(gcPath));
        ++boundaryIdx;
    }

    return gcEdgePaths;
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
