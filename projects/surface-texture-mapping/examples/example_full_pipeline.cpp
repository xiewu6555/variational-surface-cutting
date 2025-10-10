/**
 * 完整流水线示例
 * 演示变分切缝 + UV参数化 + 表面填充的完整流程
 */

#include <iostream>
#include "mesh_processing.h"
#include "variational_cutting.h"
#include "texture_mapping.h"
#include "surface_filling.h"

using namespace SurfaceTextureMapping;

int main() {
    std::cout << "=== 完整流水线示例 ===\n\n";

    try {
        // 1. 网格预处理
        std::cout << "1. 网格预处理\n";
        MeshProcessor processor;
        if (!processor.loadMesh("../data/sphere.obj")) {
            std::cerr << "无法加载网格文件\n";
            return 1;
        }

        processor.basicRepair();
        processor.isotropicRemeshing(0.02, 3, true);
        processor.makeManifold();

        auto mesh = processor.getMesh();
        auto geometry = processor.getGeometry();

        std::cout << "  预处理完成，网格: " << mesh->nVertices()
                  << " 顶点, " << mesh->nFaces() << " 面\n\n";

        // 2. 变分切缝优化
        std::cout << "2. 变分切缝优化\n";
        VariationalCutter cutter;
        cutter.setMesh(mesh, geometry);

        // 设置切缝参数
        VariationalCutter::CuttingParams cutParams;
        cutParams.lengthRegularization = 0.1;    // 适中的长度惩罚
        cutParams.smoothRegularization = 0.05;   // 平滑切缝
        cutParams.maxIterations = 50;            // 适中的迭代次数
        cutParams.timeStep = 0.01;

        auto cuts = cutter.computeOptimalCuts(cutParams);
        std::cout << "  找到 " << cuts.size() << " 条切缝\n";

        // 评估切缝质量
        auto cutQuality = cutter.evaluateCutQuality(cuts);
        std::cout << "  切缝总长度: " << cutQuality.totalLength << "\n";
        std::cout << "  平均失真: " << cutQuality.averageDistortion << "\n";

        // 应用切缝到网格
        auto cutMesh = cutter.applyCutsToMesh(cuts);
        if (cutMesh) {
            mesh = cutMesh;
            std::cout << "  切缝应用成功\n\n";
        }

        // 3. UV参数化
        std::cout << "3. UV参数化\n";
        TextureMapper mapper;
        mapper.setMesh(mesh, geometry);

        // 自动检测锥点
        auto cones = mapper.detectConeVertices(0.1);
        std::cout << "  检测到 " << cones.size() << " 个锥点\n";

        TextureMapper::MappingParams mapParams;
        mapParams.useConformalMapping = true;
        mapParams.coneVertices = cones;
        mapParams.automaticConeDetection = false; // 使用手动检测的锥点

        auto mapping = mapper.computeUVMapping(mapParams);
        if (!mapping.has_value()) {
            std::cerr << "UV参数化失败\n";
            return 1;
        }

        auto distortion = mapper.computeDistortionMetrics(mapping.value());
        std::cout << "  角度失真: " << distortion.angleDistortion << "\n";
        std::cout << "  面积失真: " << distortion.areaDistortion << "\n\n";

        // 4. 表面填充
        std::cout << "4. 表面填充\n";
        SurfaceFiller filler;
        filler.setInput(mesh, geometry, mapping.value());

        // 生成多种填充图案
        std::vector<std::pair<std::string, SurfaceFiller::PatternType>> patterns = {
            {"grid", SurfaceFiller::PatternType::Grid},
            {"hexagonal", SurfaceFiller::PatternType::Hexagonal},
            {"spiral", SurfaceFiller::PatternType::Spiral},
            {"hilbert", SurfaceFiller::PatternType::Hilbert}
        };

        for (const auto& [name, type] : patterns) {
            std::cout << "  生成 " << name << " 图案...\n";

            SurfaceFiller::FillingParams fillParams;
            fillParams.type = type;
            fillParams.spacing = (type == SurfaceFiller::PatternType::Hilbert) ? 0.1 : 0.03;
            fillParams.respectBoundary = true;

            if (type == SurfaceFiller::PatternType::Hilbert) {
                fillParams.recursionDepth = 4; // 希尔伯特曲线递归深度
            }

            auto result = filler.generateFilling(fillParams);

            std::cout << "    路径数: " << result.numPaths << "\n";
            std::cout << "    总长度: " << result.totalLength << "\n";
            std::cout << "    覆盖率: " << result.coverage << "\n";

            // 导出结果
            std::string prefix = "example_full_" + name;
            filler.exportPaths3D(result.paths3D, prefix + "_paths.obj");
            filler.exportPathsToSVG(result.pathsUV, prefix + "_pattern.svg");

            auto quality = filler.evaluateFillingQuality(result);
            std::cout << "    填充质量 - 均匀性: " << quality.uniformity
                      << ", 效率: " << quality.efficiency << "\n";
        }

        // 5. 导出最终结果
        std::cout << "\n5. 导出最终结果\n";
        mapper.exportUVMesh("example_full_final.obj", mapping.value());
        mapper.visualizeUVMapping(mapping.value(), "example_full_uv.png", 1024);

        std::cout << "  最终UV网格: example_full_final.obj\n";
        std::cout << "  UV可视化: example_full_uv.png\n";

        std::cout << "\n=== 完整流水线示例完成 ===\n";

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << "\n";
        return 1;
    }

    return 0;
}