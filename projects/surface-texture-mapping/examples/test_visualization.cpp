/**
 * 测试每一步计算结果的可视化功能
 * 验证算法计算正确性的可视化输出
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <memory>

// 核心库
#include "mesh_processor.h"
#include "variational_cutting.h"
#include "texture_mapping.h"
#include "surface_filling.h"

using namespace SurfaceTextureMapping;

// 简单的OBJ导出函数（带顶点颜色）
void exportColoredMeshToOBJ(const std::string& filename,
                             const std::vector<Vector3>& vertices,
                             const std::vector<std::array<int, 3>>& faces,
                             const std::vector<Vector3>& colors) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return;
    }

    file << "# Colored mesh with " << vertices.size() << " vertices\n";

    // 写入顶点和颜色
    for (size_t i = 0; i < vertices.size(); ++i) {
        file << "v " << vertices[i][0] << " " << vertices[i][1] << " " << vertices[i][2];
        if (i < colors.size()) {
            file << " " << colors[i][0] << " " << colors[i][1] << " " << colors[i][2];
        }
        file << "\n";
    }

    // 写入面
    for (const auto& face : faces) {
        file << "f " << (face[0] + 1) << " " << (face[1] + 1) << " " << (face[2] + 1) << "\n";
    }

    file.close();
    std::cout << "✓ 导出可视化网格到: " << filename << std::endl;
}

// 导出切割线为OBJ文件
void exportCutLinesToOBJ(const std::string& filename,
                         const std::vector<std::vector<Vector3>>& cutPaths) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return;
    }

    file << "# Cut lines with " << cutPaths.size() << " paths\n";

    int vertexOffset = 0;
    for (const auto& path : cutPaths) {
        // 写入路径顶点
        for (const auto& v : path) {
            file << "v " << v[0] << " " << v[1] << " " << v[2] << "\n";
        }

        // 写入线段
        for (size_t i = 0; i < path.size() - 1; ++i) {
            file << "l " << (vertexOffset + i + 1) << " " << (vertexOffset + i + 2) << "\n";
        }

        vertexOffset += path.size();
    }

    file.close();
    std::cout << "✓ 导出切割线到: " << filename << std::endl;
}

// 创建UV可视化图像（简单的SVG格式）
void createUVVisualizationSVG(const std::string& filename,
                               const std::vector<Vector2>& uvCoords,
                               const std::vector<std::array<int, 3>>& faces) {
    std::ofstream file(filename);
    if (!file) {
        std::cerr << "无法创建文件: " << filename << std::endl;
        return;
    }

    // SVG header
    file << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    file << "<svg width=\"1024\" height=\"1024\" xmlns=\"http://www.w3.org/2000/svg\">\n";
    file << "  <rect width=\"1024\" height=\"1024\" fill=\"white\"/>\n";

    // 绘制UV三角形
    for (const auto& face : faces) {
        if (face[0] < uvCoords.size() && face[1] < uvCoords.size() && face[2] < uvCoords.size()) {
            auto& uv0 = uvCoords[face[0]];
            auto& uv1 = uvCoords[face[1]];
            auto& uv2 = uvCoords[face[2]];

            file << "  <polygon points=\"";
            file << (uv0[0] * 1024) << "," << ((1 - uv0[1]) * 1024) << " ";
            file << (uv1[0] * 1024) << "," << ((1 - uv1[1]) * 1024) << " ";
            file << (uv2[0] * 1024) << "," << ((1 - uv2[1]) * 1024) << "\" ";
            file << "fill=\"none\" stroke=\"blue\" stroke-width=\"0.5\"/>\n";
        }
    }

    file << "</svg>\n";
    file.close();
    std::cout << "✓ 导出UV可视化到: " << filename << std::endl;
}

int main(int argc, char* argv[]) {
    std::cout << "====================================" << std::endl;
    std::cout << "    算法可视化验证测试" << std::endl;
    std::cout << "====================================" << std::endl;

    // 获取输入文件
    std::string inputFile = argc > 1 ? argv[1] : "../../../../data/spot.obj";
    std::cout << "\n输入模型: " << inputFile << std::endl;

    try {
        // 步骤1: 加载模型并可视化
        std::cout << "\n[步骤1] 加载原始模型" << std::endl;
        auto meshProcessor = std::make_unique<MeshProcessor>();
        if (!meshProcessor->loadMesh(inputFile)) {
            throw std::runtime_error("无法加载模型");
        }

        auto [vertices, faces, edges] = meshProcessor->getMeshData();
        std::cout << "  顶点: " << vertices.size() << ", 面: " << faces.size() << std::endl;

        // 可视化1: 导出原始模型
        std::vector<Vector3> originalColors(vertices.size(), Vector3{0.7, 0.7, 0.7});
        exportColoredMeshToOBJ("step1_original_mesh.obj", vertices, faces, originalColors);

        // 步骤2: 网格预处理并可视化
        std::cout << "\n[步骤2] 网格预处理" << std::endl;
        meshProcessor->preprocessMesh(0.03, 3);
        auto [preprocessedVerts, preprocessedFaces, _] = meshProcessor->getMeshData();
        std::cout << "  预处理后顶点: " << preprocessedVerts.size() << std::endl;

        // 可视化2: 导出预处理后的模型（用不同颜色）
        std::vector<Vector3> preprocessedColors(preprocessedVerts.size(), Vector3{0.5, 0.8, 0.5});
        exportColoredMeshToOBJ("step2_preprocessed_mesh.obj", preprocessedVerts, preprocessedFaces, preprocessedColors);

        // 步骤3: 变分切割并可视化切割线
        std::cout << "\n[步骤3] 变分曲面切割" << std::endl;
        auto cutter = std::make_unique<VariationalCutter>();
        cutter->setMesh(preprocessedVerts, preprocessedFaces);
        cutter->setParameters(0.1, 0.05, 100);

        auto cutPaths = cutter->computeOptimalCuts();
        std::cout << "  生成切割路径: " << cutPaths.size() << std::endl;

        // 可视化3: 导出切割线
        if (!cutPaths.empty()) {
            exportCutLinesToOBJ("step3_cut_lines.obj", cutPaths);
        }

        // 为网格着色以显示切割区域
        std::vector<Vector3> cutColors = preprocessedColors;
        // 这里可以根据切割线附近的顶点着不同颜色
        exportColoredMeshToOBJ("step3_mesh_with_cuts.obj", preprocessedVerts, preprocessedFaces, cutColors);

        // 步骤4: UV展开并可视化
        std::cout << "\n[步骤4] BFF UV展开" << std::endl;
        auto mapper = std::make_unique<TextureMapper>();
        mapper->setMesh(preprocessedVerts, preprocessedFaces);
        mapper->applyCuts(cutPaths);

        auto uvMapping = mapper->computeUVMapping();
        std::cout << "  UV坐标数: " << uvMapping.uvCoords.size() << std::endl;
        std::cout << "  失真度: " << uvMapping.distortion << std::endl;

        // 可视化4: 导出UV展开
        createUVVisualizationSVG("step4_uv_layout.svg", uvMapping.uvCoords, uvMapping.uvFaces);

        // 用UV坐标作为颜色来可视化
        std::vector<Vector3> uvColors;
        for (const auto& uv : uvMapping.uvCoords) {
            uvColors.push_back(Vector3{uv[0], uv[1], 0.5});
        }
        exportColoredMeshToOBJ("step4_mesh_with_uv.obj", preprocessedVerts, preprocessedFaces, uvColors);

        // 步骤5: Clipper2填充并可视化
        std::cout << "\n[步骤5] Clipper2图案填充" << std::endl;
        auto filler = std::make_unique<SurfaceFiller>();
        filler->setUVMapping(uvMapping.uvCoords, uvMapping.uvFaces);

        // 生成网格图案
        auto pattern = filler->generatePattern(PatternType::Grid, 0.02);
        std::cout << "  生成图案路径: " << pattern.paths.size() << std::endl;

        // 可视化5: 导出填充图案
        std::ofstream svgFile("step5_fill_pattern.svg");
        if (svgFile) {
            svgFile << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
            svgFile << "<svg width=\"1024\" height=\"1024\" xmlns=\"http://www.w3.org/2000/svg\">\n";
            svgFile << "  <rect width=\"1024\" height=\"1024\" fill=\"white\"/>\n";

            // 绘制UV边界
            for (const auto& face : uvMapping.uvFaces) {
                if (face[0] < uvMapping.uvCoords.size()) {
                    auto& uv0 = uvMapping.uvCoords[face[0]];
                    auto& uv1 = uvMapping.uvCoords[face[1]];
                    auto& uv2 = uvMapping.uvCoords[face[2]];

                    svgFile << "  <polygon points=\"";
                    svgFile << (uv0[0] * 1024) << "," << ((1 - uv0[1]) * 1024) << " ";
                    svgFile << (uv1[0] * 1024) << "," << ((1 - uv1[1]) * 1024) << " ";
                    svgFile << (uv2[0] * 1024) << "," << ((1 - uv2[1]) * 1024) << "\" ";
                    svgFile << "fill=\"none\" stroke=\"lightgray\" stroke-width=\"0.5\"/>\n";
                }
            }

            // 绘制填充图案
            for (const auto& path : pattern.paths) {
                svgFile << "  <polyline points=\"";
                for (const auto& pt : path) {
                    svgFile << (pt[0] * 1024) << "," << ((1 - pt[1]) * 1024) << " ";
                }
                svgFile << "\" fill=\"none\" stroke=\"red\" stroke-width=\"1\"/>\n";
            }

            svgFile << "</svg>\n";
            svgFile.close();
            std::cout << "✓ 导出填充图案到: step5_fill_pattern.svg" << std::endl;
        }

        // 步骤6: 映射回3D模型
        std::cout << "\n[步骤6] 纹理映射到原始模型" << std::endl;

        // 将图案映射回3D
        auto paths3D = filler->mapTo3D(preprocessedVerts, uvMapping.uvCoords);
        std::cout << "  3D路径数: " << paths3D.size() << std::endl;

        // 可视化6: 导出带纹理的最终模型
        exportCutLinesToOBJ("step6_texture_paths_3d.obj", paths3D);

        // 根据纹理密度着色
        std::vector<Vector3> textureColors(preprocessedVerts.size(), Vector3{0.3, 0.3, 0.8});
        // 这里可以根据纹理密度调整颜色
        exportColoredMeshToOBJ("step6_final_textured_mesh.obj", preprocessedVerts, preprocessedFaces, textureColors);

        std::cout << "\n====================================" << std::endl;
        std::cout << "    可视化测试完成！" << std::endl;
        std::cout << "====================================" << std::endl;
        std::cout << "\n生成的可视化文件:" << std::endl;
        std::cout << "  1. step1_original_mesh.obj - 原始模型" << std::endl;
        std::cout << "  2. step2_preprocessed_mesh.obj - 预处理后模型" << std::endl;
        std::cout << "  3. step3_cut_lines.obj - 切割线" << std::endl;
        std::cout << "  4. step4_uv_layout.svg - UV展开图" << std::endl;
        std::cout << "  5. step5_fill_pattern.svg - 填充图案" << std::endl;
        std::cout << "  6. step6_final_textured_mesh.obj - 最终纹理模型" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "错误: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}