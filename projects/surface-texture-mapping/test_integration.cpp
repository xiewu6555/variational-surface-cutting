/**
 * 集成测试程序
 * 测试真实算法集成是否正常工作
 */

#include <iostream>
#include <cassert>

// 包含集成的算法接口
#include "core/include/real_algorithm_integration.h"
#include "core/include/variational_cutting.h"
#include "core/include/texture_mapping.h"

// geometry-central用于测试网格创建
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

using namespace SurfaceTextureMapping;

/**
 * 创建一个简单的测试网格（四面体）
 */
std::pair<std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh>,
          std::shared_ptr<geometrycentral::surface::VertexPositionGeometry>>
createTestMesh() {

    // 四面体的顶点
    std::vector<geometrycentral::Vector3> vertices = {
        {0.0, 0.0, 0.0},      // 顶点0
        {1.0, 0.0, 0.0},      // 顶点1
        {0.5, 1.0, 0.0},      // 顶点2
        {0.5, 0.5, 1.0}       // 顶点3
    };

    // 四面体的面（三角形）
    std::vector<std::vector<size_t>> faces = {
        {0, 1, 2},  // 底面
        {0, 1, 3},  // 侧面1
        {1, 2, 3},  // 侧面2
        {2, 0, 3}   // 侧面3
    };

    auto mesh = std::make_shared<geometrycentral::surface::ManifoldSurfaceMesh>(faces);
    auto geometry = std::make_shared<geometrycentral::surface::VertexPositionGeometry>(*mesh, vertices);

    return {mesh, geometry};
}

/**
 * 测试算法集成状态
 */
bool testIntegrationStatus() {
    std::cout << "=== 测试集成状态 ===" << std::endl;

    auto status = RealAlgorithmIntegration::checkIntegrationStatus();
    std::cout << "集成状态: " << status.statusMessage << std::endl;

    // 基本检查
    bool hasBasicSupport = status.meshConversionSupported;
    std::cout << "网格转换支持: " << (hasBasicSupport ? "是" : "否") << std::endl;

    return hasBasicSupport;
}

/**
 * 测试变分切割器创建和基本功能
 */
bool testVariationalCutter() {
    std::cout << "\n=== 测试变分切割器 ===" << std::endl;

    try {
        // 创建集成的变分切割器
        auto cutter = RealAlgorithmIntegration::createIntegratedVariationalCutter();
        if (!cutter) {
            std::cerr << "创建变分切割器失败" << std::endl;
            return false;
        }

        // 创建测试网格
        auto [mesh, geometry] = createTestMesh();

        // 设置网格
        cutter->setMesh(mesh, geometry);

        // 设置测试参数
        VariationalCutter::CuttingParams params;
        params.lengthRegularization = 0.1;
        params.smoothRegularization = 0.05;
        params.maxIterations = 10;  // 少量迭代用于测试
        params.timeStep = 0.01;

        // 执行切割
        auto cuts = cutter->computeOptimalCuts(params);

        std::cout << "变分切割完成，生成 " << cuts.size() << " 条切缝" << std::endl;

        // 验证结果
        for (size_t i = 0; i < cuts.size(); ++i) {
            const auto& cut = cuts[i];
            std::cout << "  切缝 " << i << ": 长度=" << cut.totalLength
                      << ", 失真降低=" << cut.distortionReduction << std::endl;
        }

        // 评估切缝质量
        auto quality = cutter->evaluateCutQuality(cuts);
        std::cout << "切缝质量评估完成" << std::endl;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "变分切割器测试失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * 测试纹理映射器创建和基本功能
 */
bool testTextureMapper() {
    std::cout << "\n=== 测试纹理映射器 ===" << std::endl;

    try {
        // 创建集成的纹理映射器
        auto mapper = RealAlgorithmIntegration::createIntegratedTextureMapper();
        if (!mapper) {
            std::cerr << "创建纹理映射器失败" << std::endl;
            return false;
        }

        // 创建测试网格
        auto [mesh, geometry] = createTestMesh();

        // 设置网格
        mapper->setMesh(mesh, geometry);

        // 设置测试参数
        TextureMapper::MappingParams params;
        params.useConformalMapping = true;
        params.enableAreaCorrection = false;
        params.boundaryWeight = 1.0;
        params.automaticConeDetection = true;

        // 执行UV映射
        auto uvMapping = mapper->computeUVMapping(params);

        if (!uvMapping.has_value()) {
            std::cerr << "UV映射失败" << std::endl;
            return false;
        }

        std::cout << "UV映射完成:" << std::endl;
        std::cout << "  UV坐标数: " << uvMapping->uvCoordinates.size() << std::endl;
        std::cout << "  图块数: " << uvMapping->charts.size() << std::endl;
        std::cout << "  总失真: " << uvMapping->totalDistortion << std::endl;

        // 计算失真度量
        auto distortion = mapper->computeDistortionMetrics(uvMapping.value());
        std::cout << "失真度量计算完成" << std::endl;

        return true;

    } catch (const std::exception& e) {
        std::cerr << "纹理映射器测试失败: " << e.what() << std::endl;
        return false;
    }
}

/**
 * 测试完整流水线
 */
bool testFullPipeline() {
    std::cout << "\n=== 测试完整流水线 ===" << std::endl;

    try {
        // 创建测试网格
        auto [mesh, geometry] = createTestMesh();

        // 设置参数
        VariationalCutter::CuttingParams cuttingParams;
        cuttingParams.lengthRegularization = 0.1;
        cuttingParams.maxIterations = 5;

        TextureMapper::MappingParams mappingParams;
        mappingParams.useConformalMapping = true;

        // 运行完整流水线
        bool success = RealAlgorithmIntegration::processFullPipeline(
            mesh, geometry,
            cuttingParams, mappingParams,
            "test_output.obj"
        );

        if (success) {
            std::cout << "完整流水线测试成功" << std::endl;
        } else {
            std::cerr << "完整流水线测试失败" << std::endl;
        }

        return success;

    } catch (const std::exception& e) {
        std::cerr << "完整流水线测试异常: " << e.what() << std::endl;
        return false;
    }
}

/**
 * 主测试函数
 */
int main() {
    std::cout << "=== Surface Texture Mapping 算法集成测试 ===" << std::endl;

    int passedTests = 0;
    int totalTests = 4;

    // 测试1: 集成状态
    if (testIntegrationStatus()) {
        passedTests++;
        std::cout << "✓ 集成状态测试通过" << std::endl;
    } else {
        std::cout << "✗ 集成状态测试失败" << std::endl;
    }

    // 测试2: 变分切割器
    if (testVariationalCutter()) {
        passedTests++;
        std::cout << "✓ 变分切割器测试通过" << std::endl;
    } else {
        std::cout << "✗ 变分切割器测试失败" << std::endl;
    }

    // 测试3: 纹理映射器
    if (testTextureMapper()) {
        passedTests++;
        std::cout << "✓ 纹理映射器测试通过" << std::endl;
    } else {
        std::cout << "✗ 纹理映射器测试失败" << std::endl;
    }

    // 测试4: 完整流水线
    if (testFullPipeline()) {
        passedTests++;
        std::cout << "✓ 完整流水线测试通过" << std::endl;
    } else {
        std::cout << "✗ 完整流水线测试失败" << std::endl;
    }

    // 总结
    std::cout << "\n=== 测试总结 ===" << std::endl;
    std::cout << "通过测试: " << passedTests << "/" << totalTests << std::endl;

    if (passedTests == totalTests) {
        std::cout << "🎉 所有测试通过！算法集成成功！" << std::endl;
        return 0;
    } else {
        std::cout << "⚠️  部分测试失败，请检查集成实现" << std::endl;
        return 1;
    }
}