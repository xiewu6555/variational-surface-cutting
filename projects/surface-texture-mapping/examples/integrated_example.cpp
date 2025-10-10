/**
 * 集成算法示例
 * 演示如何使用真实的EulerianShapeOptimizer和BFF算法
 */

#include <iostream>
#include <memory>
#include <string>

// geometry-central
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

// 集成的算法
#include "real_algorithm_integration.h"
#include "variational_cutting.h"
#include "texture_mapping.h"

using namespace SurfaceTextureMapping;

int main(int argc, char* argv[]) {
    std::cout << "=== Surface Texture Mapping 集成算法示例 ===" << std::endl;

    // 检查命令行参数
    if (argc < 2) {
        std::cout << "用法: " << argv[0] << " <网格文件路径> [输出路径]" << std::endl;
        std::cout << "例如: " << argv[0] << " ../data/spot.obj output/" << std::endl;
        return 1;
    }

    std::string inputPath = argv[1];
    std::string outputPath = (argc > 2) ? argv[2] : "output.obj";

    std::cout << "输入网格: " << inputPath << std::endl;
    std::cout << "输出路径: " << outputPath << std::endl;

    try {
        // 1. 加载网格
        std::cout << "\n=== 步骤1: 加载网格 ===" << std::endl;
        std::unique_ptr<geometrycentral::surface::ManifoldSurfaceMesh> mesh;
        std::unique_ptr<geometrycentral::surface::VertexPositionGeometry> geometry;

        // 尝试加载网格文件
        try {
            std::tie(mesh, geometry) = geometrycentral::surface::readManifoldSurfaceMesh(inputPath);
            std::cout << "成功加载网格:" << std::endl;
            std::cout << "  顶点数: " << mesh->nVertices() << std::endl;
            std::cout << "  面数: " << mesh->nFaces() << std::endl;
            std::cout << "  边数: " << mesh->nEdges() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "加载网格失败: " << e.what() << std::endl;
            std::cout << "使用默认测试网格..." << std::endl;

            // 创建一个简单的测试网格（三角形）
            std::vector<std::vector<size_t>> faceIndices = {{0, 1, 2}};
            std::vector<geometrycentral::Vector3> vertexPositions = {
                {0.0, 0.0, 0.0},
                {1.0, 0.0, 0.0},
                {0.5, 1.0, 0.0}
            };

            mesh = std::make_unique<geometrycentral::surface::ManifoldSurfaceMesh>(faceIndices);
            geometry = std::make_unique<geometrycentral::surface::VertexPositionGeometry>(*mesh, vertexPositions);

            std::cout << "创建了测试网格 (三角形)" << std::endl;
        }

        // 2. 检查集成状态
        std::cout << "\n=== 步骤2: 检查算法集成状态 ===" << std::endl;
        auto status = RealAlgorithmIntegration::checkIntegrationStatus();
        std::cout << status.statusMessage << std::endl;

        // 3. 变分切割
        std::cout << "\n=== 步骤3: 变分切割 ===" << std::endl;
        auto cutter = RealAlgorithmIntegration::createIntegratedVariationalCutter();
        if (!cutter) {
            std::cerr << "创建变分切割器失败" << std::endl;
            return 1;
        }

        // 转换为共享指针
        auto meshPtr = std::shared_ptr<geometrycentral::surface::ManifoldSurfaceMesh>(std::move(mesh));
        auto geometryPtr = std::shared_ptr<geometrycentral::surface::VertexPositionGeometry>(std::move(geometry));

        cutter->setMesh(meshPtr, geometryPtr);

        // 设置切割参数
        VariationalCutter::CuttingParams cuttingParams;
        cuttingParams.lengthRegularization = 0.1;
        cuttingParams.smoothRegularization = 0.05;
        cuttingParams.maxIterations = 50;
        cuttingParams.timeStep = 0.01;

        // 执行切割
        auto cuts = cutter->computeOptimalCuts(cuttingParams);
        std::cout << "生成了 " << cuts.size() << " 条切缝" << std::endl;

        // 应用切缝到网格
        auto cutMesh = cutter->applyCutsToMesh(cuts);
        if (!cutMesh) {
            std::cerr << "应用切缝失败" << std::endl;
            return 1;
        }

        // 4. UV参数化
        std::cout << "\n=== 步骤4: UV参数化 ===" << std::endl;
        auto mapper = RealAlgorithmIntegration::createIntegratedTextureMapper();
        if (!mapper) {
            std::cerr << "创建纹理映射器失败" << std::endl;
            return 1;
        }

        mapper->setMesh(cutMesh, geometryPtr);

        // 设置映射参数
        TextureMapper::MappingParams mappingParams;
        mappingParams.useConformalMapping = true;
        mappingParams.enableAreaCorrection = false;
        mappingParams.boundaryWeight = 1.0;
        mappingParams.automaticConeDetection = true;

        // 执行UV映射
        auto uvMapping = mapper->computeUVMapping(mappingParams);
        if (!uvMapping.has_value()) {
            std::cerr << "UV映射失败" << std::endl;
            return 1;
        }

        std::cout << "UV映射成功:" << std::endl;
        std::cout << "  UV坐标数: " << uvMapping->uvCoordinates.size() << std::endl;
        std::cout << "  图块数: " << uvMapping->charts.size() << std::endl;
        std::cout << "  总失真: " << uvMapping->totalDistortion << std::endl;

        // 5. 评估结果质量
        std::cout << "\n=== 步骤5: 评估结果质量 ===" << std::endl;

        // 评估切缝质量
        auto cutQuality = cutter->evaluateCutQuality(cuts);
        std::cout << "切缝质量评估:" << std::endl;
        std::cout << "  总长度: " << cutQuality.totalLength << std::endl;
        std::cout << "  平均失真: " << cutQuality.averageDistortion << std::endl;
        std::cout << "  最大失真: " << cutQuality.maxDistortion << std::endl;

        // 评估UV映射质量
        auto distortionMetrics = mapper->computeDistortionMetrics(uvMapping.value());
        std::cout << "UV映射质量评估:" << std::endl;
        std::cout << "  角度失真: " << distortionMetrics.angleDistortion << std::endl;
        std::cout << "  面积失真: " << distortionMetrics.areaDistortion << std::endl;
        std::cout << "  共形误差: " << distortionMetrics.conformalError << std::endl;

        // 6. 导出结果
        std::cout << "\n=== 步骤6: 导出结果 ===" << std::endl;
        bool exportSuccess = mapper->exportUVMesh(outputPath, uvMapping.value());
        if (exportSuccess) {
            std::cout << "结果已导出到: " << outputPath << std::endl;
        } else {
            std::cerr << "导出失败" << std::endl;
            return 1;
        }

        // 7. 运行集成测试
        std::cout << "\n=== 步骤7: 运行集成测试 ===" << std::endl;
        bool testSuccess = RealAlgorithmIntegration::runIntegrationTests(inputPath);
        if (testSuccess) {
            std::cout << "集成测试通过" << std::endl;
        } else {
            std::cout << "集成测试失败" << std::endl;
        }

        std::cout << "\n=== 示例运行完成 ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "运行时错误: " << e.what() << std::endl;
        return 1;
    }
}

/**
 * 辅助函数：打印算法性能统计
 */
void printPerformanceStats(const std::vector<VariationalCutter::CutCurve>& cuts,
                          const TextureMapper::UVMapping& mapping) {
    std::cout << "\n=== 性能统计 ===" << std::endl;

    double totalCutLength = 0.0;
    for (const auto& cut : cuts) {
        totalCutLength += cut.totalLength;
    }

    std::cout << "切缝统计:" << std::endl;
    std::cout << "  切缝数量: " << cuts.size() << std::endl;
    std::cout << "  总长度: " << totalCutLength << std::endl;

    std::cout << "UV映射统计:" << std::endl;
    std::cout << "  图块数量: " << mapping.charts.size() << std::endl;
    std::cout << "  总失真: " << mapping.totalDistortion << std::endl;
    std::cout << "  最大失真: " << mapping.maxDistortion << std::endl;

    // 计算打包效率
    if (!mapping.charts.empty()) {
        double averageChartSize = static_cast<double>(mapping.uvCoordinates.size()) / mapping.charts.size();
        std::cout << "  平均图块大小: " << averageChartSize << " 顶点" << std::endl;
    }
}