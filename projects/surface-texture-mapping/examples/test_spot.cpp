/**
 * Spot模型测试程序
 * 使用spot.obj模型测试完整的纹理映射流水线
 */

#include <iostream>
#include <chrono>
#include <filesystem>

#include "../core/include/mesh_processing.h"
#include "../core/include/variational_cutting.h"
#include "../core/include/texture_mapping.h"
#include "../core/include/surface_filling.h"

namespace fs = std::filesystem;
using namespace SurfaceTextureMapping;

// 计时器辅助类
class Timer {
    std::chrono::high_resolution_clock::time_point start_;
    std::string name_;
public:
    Timer(const std::string& name) : name_(name) {
        start_ = std::chrono::high_resolution_clock::now();
    }
    ~Timer() {
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start_);
        std::cout << name_ << " 耗时: " << duration.count() << " ms\n";
    }
};

void printSeparator() {
    std::cout << "========================================\n";
}

int main(int argc, char* argv[]) {
    std::cout << "=== Spot模型纹理映射测试 ===\n\n";

    // 检查spot.obj文件是否存在
    std::string spotFile = "data/spot.obj";
    if (argc > 1) {
        spotFile = argv[1];
    }

    if (!fs::exists(spotFile)) {
        std::cerr << "错误: 找不到文件 " << spotFile << "\n";
        std::cerr << "请确保在项目根目录运行，或提供正确的文件路径\n";
        return 1;
    }

    try {
        // ====== 1. 网格预处理 ======
        printSeparator();
        std::cout << "步骤 1: 网格预处理\n";
        printSeparator();

        MeshProcessor processor;
        {
            Timer t("加载spot.obj");
            if (!processor.loadMesh(spotFile)) {
                std::cerr << "错误: 无法加载spot.obj\n";
                return 1;
            }
        }

        // 打印原始网格信息
        auto stats = processor.getMeshStats();
        std::cout << "\n原始网格统计:\n";
        std::cout << "  顶点数: " << stats.numVertices << "\n";
        std::cout << "  面数: " << stats.numFaces << "\n";
        std::cout << "  边数: " << stats.numEdges << "\n";
        std::cout << "  平均边长: " << stats.avgEdgeLength << "\n";
        std::cout << "  最小边长: " << stats.minEdgeLength << "\n";
        std::cout << "  最大边长: " << stats.maxEdgeLength << "\n";
        std::cout << "  流形状态: " << (stats.isManifold ? "是" : "否") << "\n";
        std::cout << "  封闭状态: " << (stats.isClosed ? "是" : "否") << "\n\n";

        // 网格修复
        {
            Timer t("基础修复");
            processor.basicRepair();
        }

        // 等各向性重网格化
        double targetLength = stats.avgEdgeLength * 0.8; // 稍微细化
        std::cout << "目标边长: " << targetLength << "\n";
        {
            Timer t("等各向性重网格化");
            processor.isotropicRemeshing(targetLength, 3, true);
        }

        // 流形化
        {
            Timer t("流形化处理");
            processor.makeManifold();
        }

        // 打印处理后的网格信息
        stats = processor.getMeshStats();
        std::cout << "\n处理后网格统计:\n";
        std::cout << "  顶点数: " << stats.numVertices << "\n";
        std::cout << "  面数: " << stats.numFaces << "\n";
        std::cout << "  边数: " << stats.numEdges << "\n";
        std::cout << "  平均边长: " << stats.avgEdgeLength << "\n";
        std::cout << "  流形状态: " << (stats.isManifold ? "是" : "否") << "\n\n";

        auto mesh = processor.getMesh();
        auto geometry = processor.getGeometry();

        // ====== 2. 变分切缝 ======
        printSeparator();
        std::cout << "步骤 2: 变分切缝优化\n";
        printSeparator();

        VariationalCutter cutter;
        cutter.setMesh(mesh, geometry);

        // 设置切缝参数
        VariationalCutter::CuttingParams cutParams;
        cutParams.lengthRegularization = 0.1;    // 适中的长度惩罚
        cutParams.smoothRegularization = 0.05;   // 平滑切缝
        cutParams.maxIterations = 30;            // 减少迭代次数以加快测试
        cutParams.timeStep = 0.01;
        cutParams.convergenceThreshold = 1e-5;

        std::cout << "切缝参数:\n";
        std::cout << "  长度正则化: " << cutParams.lengthRegularization << "\n";
        std::cout << "  平滑正则化: " << cutParams.smoothRegularization << "\n";
        std::cout << "  最大迭代次数: " << cutParams.maxIterations << "\n\n";

        std::vector<VariationalCutter::CutCurve> cuts;
        {
            Timer t("变分切缝计算");
            cuts = cutter.computeOptimalCuts(cutParams);
        }

        std::cout << "\n切缝结果:\n";
        std::cout << "  找到 " << cuts.size() << " 条切缝\n";

        // 评估切缝质量
        auto cutQuality = cutter.evaluateCutQuality(cuts);
        std::cout << "  总长度: " << cutQuality.totalLength << "\n";
        std::cout << "  平均失真: " << cutQuality.averageDistortion << "\n";
        std::cout << "  最大失真: " << cutQuality.maxDistortion << "\n";
        std::cout << "  可见性评分: " << cutQuality.visibilityScore << "\n\n";

        // 应用切缝到网格
        auto cutMesh = cutter.applyCutsToMesh(cuts);
        if (cutMesh) {
            mesh = cutMesh;
            std::cout << "切缝应用成功\n\n";
        }

        // ====== 3. UV参数化 ======
        printSeparator();
        std::cout << "步骤 3: UV参数化 (BFF)\n";
        printSeparator();

        TextureMapper mapper;
        mapper.setMesh(mesh, geometry);

        // 自动检测锥点
        std::cout << "检测锥点...\n";
        auto cones = mapper.detectConeVertices(0.15);
        std::cout << "  检测到 " << cones.size() << " 个锥点\n\n";

        // 设置映射参数
        TextureMapper::MappingParams mapParams;
        mapParams.useConformalMapping = true;
        mapParams.coneVertices = cones;
        mapParams.automaticConeDetection = false; // 使用手动检测的锥点
        mapParams.enableAreaCorrection = false;

        TextureMapper::UVMapping uvMapping;
        {
            Timer t("BFF参数化计算");
            auto result = mapper.computeUVMapping(mapParams);
            if (!result.has_value()) {
                std::cerr << "错误: UV参数化失败\n";
                return 1;
            }
            uvMapping = result.value();
        }

        std::cout << "\nUV映射结果:\n";
        std::cout << "  图块数: " << uvMapping.charts.size() << "\n";
        std::cout << "  总失真: " << uvMapping.totalDistortion << "\n";
        std::cout << "  最大失真: " << uvMapping.maxDistortion << "\n";

        // 计算详细的失真指标
        auto distortion = mapper.computeDistortionMetrics(uvMapping);
        std::cout << "\n失真分析:\n";
        std::cout << "  角度失真: " << distortion.angleDistortion << "\n";
        std::cout << "  面积失真: " << distortion.areaDistortion << "\n";
        std::cout << "  共形误差: " << distortion.conformalError << "\n\n";

        // 导出UV网格
        {
            Timer t("导出UV网格");
            mapper.exportUVMesh("data/spot_uv.obj", uvMapping);
            std::cout << "UV网格已保存到: data/spot_uv.obj\n";
        }

        // 可视化UV映射
        {
            Timer t("生成UV可视化");
            mapper.visualizeUVMapping(uvMapping, "data/spot_uv.png", 2048);
            std::cout << "UV可视化已保存到: data/spot_uv.png\n\n";
        }

        // ====== 4. 表面填充测试 ======
        printSeparator();
        std::cout << "步骤 4: 表面填充图案生成\n";
        printSeparator();

        SurfaceFiller filler;
        filler.setInput(mesh, geometry, uvMapping);

        // 测试多种填充图案
        std::vector<std::pair<std::string, SurfaceFiller::PatternType>> patterns = {
            {"grid", SurfaceFiller::PatternType::Grid},
            {"hexagonal", SurfaceFiller::PatternType::Hexagonal},
            {"hilbert", SurfaceFiller::PatternType::Hilbert}
        };

        for (const auto& [name, type] : patterns) {
            std::cout << "\n生成 " << name << " 图案...\n";

            SurfaceFiller::FillingParams fillParams;
            fillParams.type = type;
            fillParams.spacing = (type == SurfaceFiller::PatternType::Hilbert) ? 0.05 : 0.02;
            fillParams.respectBoundary = true;
            fillParams.lineWidth = 0.001;

            if (type == SurfaceFiller::PatternType::Hilbert) {
                fillParams.recursionDepth = 3; // 希尔伯特曲线递归深度
            }

            SurfaceFiller::FillingResult result;
            {
                Timer t("  图案生成");
                result = filler.generateFilling(fillParams);
            }

            std::cout << "  结果:\n";
            std::cout << "    路径数: " << result.numPaths << "\n";
            std::cout << "    总长度: " << result.totalLength << "\n";
            std::cout << "    覆盖率: " << result.coverage * 100 << "%\n";

            // 评估填充质量
            auto quality = filler.evaluateFillingQuality(result);
            std::cout << "  质量评估:\n";
            std::cout << "    均匀性: " << quality.uniformity << "\n";
            std::cout << "    效率: " << quality.efficiency << "\n";
            std::cout << "    边界遵循: " << quality.boundaryRespect << "\n";
            std::cout << "    平滑度: " << quality.smoothness << "\n";

            // 导出结果
            std::string prefix = "data/spot_" + name;
            {
                Timer t("  导出文件");
                filler.exportPaths3D(result.paths3D, prefix + "_paths.obj");
                filler.exportPathsToSVG(result.pathsUV, prefix + "_pattern.svg", 2.0);
            }

            std::cout << "  文件已保存:\n";
            std::cout << "    3D路径: " << prefix << "_paths.obj\n";
            std::cout << "    SVG图案: " << prefix << "_pattern.svg\n";
        }

        // ====== 5. 最终统计 ======
        printSeparator();
        std::cout << "测试完成 - 总体统计\n";
        printSeparator();

        std::cout << "\n生成的所有文件:\n";
        std::cout << "  1. data/spot_uv.obj - 带UV坐标的网格\n";
        std::cout << "  2. data/spot_uv.png - UV映射可视化\n";
        std::cout << "  3. data/spot_grid_paths.obj - 网格图案3D路径\n";
        std::cout << "  4. data/spot_grid_pattern.svg - 网格图案SVG\n";
        std::cout << "  5. data/spot_hexagonal_paths.obj - 六边形图案3D路径\n";
        std::cout << "  6. data/spot_hexagonal_pattern.svg - 六边形图案SVG\n";
        std::cout << "  7. data/spot_hilbert_paths.obj - 希尔伯特曲线3D路径\n";
        std::cout << "  8. data/spot_hilbert_pattern.svg - 希尔伯特曲线SVG\n";

        std::cout << "\n=== 所有测试成功完成 ===\n";

    } catch (const std::exception& e) {
        std::cerr << "\n错误: " << e.what() << "\n";
        return 1;
    }

    return 0;
}