#include <iostream>
#include <string>
#include <memory>

#include "mesh_processing.h"
#include "variational_cutting.h"
#include "texture_mapping.h"
#include "surface_filling.h"

using namespace SurfaceTextureMapping;

void printUsage() {
    std::cout << "曲面纹理映射工具\n";
    std::cout << "用法: SurfaceTextureMapping [选项] <输入网格文件>\n\n";
    std::cout << "选项:\n";
    std::cout << "  -h, --help              显示帮助信息\n";
    std::cout << "  -o, --output <文件>     输出文件前缀 (默认: output)\n";
    std::cout << "  -r, --remesh <长度>     等各向性重网格目标边长\n";
    std::cout << "  -c, --cutting           启用变分切缝\n";
    std::cout << "  -t, --texture           生成纹理映射\n";
    std::cout << "  -f, --fill <类型>       生成填充图案 (grid|hex|spiral|hilbert)\n";
    std::cout << "  --spacing <值>          填充图案间距 (默认: 0.05)\n";
    std::cout << "  --iterations <值>       变分切缝迭代次数 (默认: 100)\n";
    std::cout << "  --length-reg <值>       长度正则化权重 (默认: 0.1)\n";
    std::cout << "  --smooth-reg <值>       平滑正则化权重 (默认: 0.05)\n\n";
    std::cout << "示例:\n";
    std::cout << "  # 基础网格处理和纹理映射\n";
    std::cout << "  SurfaceTextureMapping -r 0.01 -t bunny.obj\n\n";
    std::cout << "  # 完整流水线：重网格+变分切缝+纹理映射+网格填充\n";
    std::cout << "  SurfaceTextureMapping -r 0.01 -c -t -f grid --spacing 0.02 bunny.obj\n";
}

struct ProgramOptions {
    std::string inputFile;
    std::string outputPrefix = "output";
    std::optional<double> remeshEdgeLength;
    bool enableCutting = false;
    bool enableTexture = false;
    std::string fillPattern = "";
    double fillSpacing = 0.05;
    int cuttingIterations = 100;
    double lengthReg = 0.1;
    double smoothReg = 0.05;
};

ProgramOptions parseArguments(int argc, char* argv[]) {
    ProgramOptions options;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];

        if (arg == "-h" || arg == "--help") {
            printUsage();
            exit(0);
        } else if (arg == "-o" || arg == "--output") {
            if (i + 1 < argc) options.outputPrefix = argv[++i];
        } else if (arg == "-r" || arg == "--remesh") {
            if (i + 1 < argc) options.remeshEdgeLength = std::stod(argv[++i]);
        } else if (arg == "-c" || arg == "--cutting") {
            options.enableCutting = true;
        } else if (arg == "-t" || arg == "--texture") {
            options.enableTexture = true;
        } else if (arg == "-f" || arg == "--fill") {
            if (i + 1 < argc) options.fillPattern = argv[++i];
        } else if (arg == "--spacing") {
            if (i + 1 < argc) options.fillSpacing = std::stod(argv[++i]);
        } else if (arg == "--iterations") {
            if (i + 1 < argc) options.cuttingIterations = std::stoi(argv[++i]);
        } else if (arg == "--length-reg") {
            if (i + 1 < argc) options.lengthReg = std::stod(argv[++i]);
        } else if (arg == "--smooth-reg") {
            if (i + 1 < argc) options.smoothReg = std::stod(argv[++i]);
        } else if (arg[0] != '-') {
            options.inputFile = arg;
        }
    }

    return options;
}

SurfaceFiller::PatternType parsePatternType(const std::string& pattern) {
    if (pattern == "grid") return SurfaceFiller::PatternType::Grid;
    if (pattern == "hex") return SurfaceFiller::PatternType::Hexagonal;
    if (pattern == "spiral") return SurfaceFiller::PatternType::Spiral;
    if (pattern == "hilbert") return SurfaceFiller::PatternType::Hilbert;
    return SurfaceFiller::PatternType::Grid;
}

int main(int argc, char* argv[]) {
    std::cout << "=== 曲面纹理映射工具 v1.0 ===\n\n";

    if (argc < 2) {
        printUsage();
        return 1;
    }

    ProgramOptions options = parseArguments(argc, argv);

    if (options.inputFile.empty()) {
        std::cerr << "错误: 请指定输入网格文件\n";
        return 1;
    }

    try {
        // 1. 网格预处理
        std::cout << "1. 加载和预处理网格...\n";
        MeshProcessor processor;

        if (!processor.loadMesh(options.inputFile)) {
            std::cerr << "错误: 无法加载网格文件 " << options.inputFile << "\n";
            return 1;
        }

        auto stats = processor.getMeshStats();
        std::cout << "   网格统计: " << stats.numVertices << " 顶点, "
                  << stats.numFaces << " 面, " << stats.numEdges << " 边\n";
        std::cout << "   流形状态: " << (stats.isManifold ? "是" : "否") << "\n";

        // 基础修复
        if (!processor.basicRepair()) {
            std::cerr << "警告: 网格基础修复可能不完整\n";
        }

        // 重网格化
        if (options.remeshEdgeLength.has_value()) {
            std::cout << "   执行等各向性重网格化 (目标边长: "
                      << options.remeshEdgeLength.value() << ")...\n";
            processor.isotropicRemeshing(options.remeshEdgeLength.value());

            stats = processor.getMeshStats();
            std::cout << "   重网格后: " << stats.numVertices << " 顶点, "
                      << stats.numFaces << " 面\n";
        }

        // 流形化
        if (!stats.isManifold) {
            std::cout << "   执行流形化处理...\n";
            processor.makeManifold();
        }

        auto mesh = processor.getMesh();
        auto geometry = processor.getGeometry();

        // 2. 变分切缝 (可选)
        std::vector<VariationalCutter::CutCurve> cuts;
        if (options.enableCutting) {
            std::cout << "\n2. 执行变分切缝优化...\n";

            VariationalCutter cutter;
            cutter.setMesh(mesh, geometry);

            VariationalCutter::CuttingParams params;
            params.lengthRegularization = options.lengthReg;
            params.smoothRegularization = options.smoothReg;
            params.maxIterations = options.cuttingIterations;

            cuts = cutter.computeOptimalCuts(params);

            auto quality = cutter.evaluateCutQuality(cuts);
            std::cout << "   切缝质量: 总长度=" << quality.totalLength
                      << ", 平均失真=" << quality.averageDistortion
                      << ", 切缝数量=" << quality.numCuts << "\n";

            // 应用切缝到网格
            auto cutMesh = cutter.applyCutsToMesh(cuts);
            if (cutMesh) {
                mesh = cutMesh;
                // 需要重新计算几何信息
                // geometry = ...  // 这里需要根据切开的网格重新构建几何
            }
        }

        // 3. 纹理映射
        TextureMapper::UVMapping uvMapping;
        if (options.enableTexture) {
            std::cout << "\n3. 计算UV参数化...\n";

            TextureMapper mapper;
            mapper.setMesh(mesh, geometry);

            TextureMapper::MappingParams params;
            params.automaticConeDetection = true;
            params.curvatureThreshold = 0.1;

            auto mappingResult = mapper.computeUVMapping(params);
            if (!mappingResult.has_value()) {
                std::cerr << "错误: UV参数化失败\n";
                return 1;
            }

            uvMapping = mappingResult.value();

            auto distortion = mapper.computeDistortionMetrics(uvMapping);
            std::cout << "   失真指标: 角度失真=" << distortion.angleDistortion
                      << ", 面积失真=" << distortion.areaDistortion << "\n";

            // 导出UV网格
            std::string uvFile = options.outputPrefix + "_uv.obj";
            mapper.exportUVMesh(uvFile, uvMapping);
            std::cout << "   UV网格已保存到: " << uvFile << "\n";

            // 可视化UV映射
            std::string uvImage = options.outputPrefix + "_uv.png";
            mapper.visualizeUVMapping(uvMapping, uvImage, 1024);
            std::cout << "   UV可视化已保存到: " << uvImage << "\n";
        }

        // 4. 曲面填充 (可选)
        if (!options.fillPattern.empty() && options.enableTexture) {
            std::cout << "\n4. 生成表面填充图案...\n";

            SurfaceFiller filler;
            filler.setInput(mesh, geometry, uvMapping);

            SurfaceFiller::FillingParams fillParams;
            fillParams.type = parsePatternType(options.fillPattern);
            fillParams.spacing = options.fillSpacing;
            fillParams.respectBoundary = true;

            auto fillResult = filler.generateFilling(fillParams);

            std::cout << "   填充结果: " << fillResult.numPaths << " 条路径, "
                      << "总长度=" << fillResult.totalLength
                      << ", 覆盖率=" << fillResult.coverage << "\n";

            // 导出填充路径
            std::string pathsFile = options.outputPrefix + "_paths.obj";
            filler.exportPaths3D(fillResult.paths3D, pathsFile);
            std::cout << "   3D路径已保存到: " << pathsFile << "\n";

            std::string svgFile = options.outputPrefix + "_pattern.svg";
            filler.exportPathsToSVG(fillResult.pathsUV, svgFile);
            std::cout << "   UV图案已保存到: " << svgFile << "\n";

            auto quality = filler.evaluateFillingQuality(fillResult);
            std::cout << "   填充质量: 均匀性=" << quality.uniformity
                      << ", 效率=" << quality.efficiency << "\n";
        }

        std::cout << "\n=== 处理完成 ===\n";

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << "\n";
        return 1;
    }

    return 0;
}