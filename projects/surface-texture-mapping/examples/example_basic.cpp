/**
 * 基础纹理映射示例
 * 演示如何进行网格预处理和UV参数化
 */

#include <iostream>
#include "mesh_processing.h"
#include "texture_mapping.h"

using namespace SurfaceTextureMapping;

int main() {
    std::cout << "=== 基础纹理映射示例 ===\n\n";

    // 1. 加载网格
    MeshProcessor processor;
    if (!processor.loadMesh("../data/bunny.obj")) {
        std::cerr << "无法加载网格文件\n";
        return 1;
    }

    std::cout << "原始网格统计:\n";
    auto stats = processor.getMeshStats();
    std::cout << "  顶点数: " << stats.numVertices << "\n";
    std::cout << "  面数: " << stats.numFaces << "\n";
    std::cout << "  是否为流形: " << (stats.isManifold ? "是" : "否") << "\n";
    std::cout << "  平均边长: " << stats.avgEdgeLength << "\n\n";

    // 2. 网格预处理
    std::cout << "执行网格预处理...\n";

    // 基础修复
    processor.basicRepair();

    // 等各向性重网格化
    double targetLength = stats.avgEdgeLength * 0.8; // 稍微细化
    processor.isotropicRemeshing(targetLength, 3, true);

    // 确保为流形
    processor.makeManifold();

    std::cout << "预处理后网格统计:\n";
    stats = processor.getMeshStats();
    std::cout << "  顶点数: " << stats.numVertices << "\n";
    std::cout << "  面数: " << stats.numFaces << "\n";
    std::cout << "  是否为流形: " << (stats.isManifold ? "是" : "否") << "\n\n";

    // 3. UV参数化
    std::cout << "计算UV参数化...\n";

    TextureMapper mapper;
    mapper.setMesh(processor.getMesh(), processor.getGeometry());

    // 配置参数化参数
    TextureMapper::MappingParams params;
    params.useConformalMapping = true;
    params.automaticConeDetection = true;
    params.curvatureThreshold = 0.15; // 较宽松的锥点检测

    auto mapping = mapper.computeUVMapping(params);
    if (!mapping.has_value()) {
        std::cerr << "UV参数化失败\n";
        return 1;
    }

    // 4. 评估映射质量
    std::cout << "UV映射质量评估:\n";
    auto distortion = mapper.computeDistortionMetrics(mapping.value());
    std::cout << "  角度失真: " << distortion.angleDistortion << "\n";
    std::cout << "  面积失真: " << distortion.areaDistortion << "\n";
    std::cout << "  共形误差: " << distortion.conformalError << "\n\n";

    // 5. 导出结果
    std::cout << "导出结果文件...\n";

    // 导出UV网格
    mapper.exportUVMesh("example_basic_uv.obj", mapping.value());
    std::cout << "  UV网格: example_basic_uv.obj\n";

    // 可视化UV映射
    mapper.visualizeUVMapping(mapping.value(), "example_basic_uv.png", 1024);
    std::cout << "  UV可视化: example_basic_uv.png\n";

    std::cout << "\n=== 示例完成 ===\n";
    return 0;
}