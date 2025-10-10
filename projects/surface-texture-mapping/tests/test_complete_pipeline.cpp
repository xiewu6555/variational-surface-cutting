/**
 * 完整流水线测试程序
 * 实现从Spot模型加载到最终纹理映射的完整流程
 */

#include <iostream>
#include <memory>
#include <string>
#include <vector>

// 核心库
#include "texture_mapping.h"
#include "mesh_processor.h"
#include "variational_cutting.h"
#include "bff_wrapper.h"

// Geometry central
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

// Clipper2库
#include "clipper2/clipper.h"

using namespace SurfaceTextureMapping;
namespace gc = geometrycentral;
namespace surface = geometrycentral::surface;

/**
 * Clipper2 Offset填充实现
 */
class PatternFiller {
public:
    using Path = std::vector<geometrycentral::Vector2>;
    using Paths = std::vector<Path>;

    struct FillPattern {
        Paths outlines;      // 外轮廓
        Paths fillLines;     // 填充线条
        double spacing;      // 线条间距
        double offsetAmount; // 偏移量
    };

    /**
     * 使用Clipper2创建偏移填充图案
     */
    static FillPattern createOffsetPattern(const Paths& boundaries, double spacing, int iterations) {
        FillPattern pattern;
        pattern.outlines = boundaries;
        pattern.spacing = spacing;
        pattern.offsetAmount = spacing;

        using namespace Clipper2Lib;

        // 转换到Clipper格式（缩放到整数）
        const double scale = 10000.0;
        Paths64 clipperPaths;

        for (const auto& boundary : boundaries) {
            Path64 path;
            for (const auto& pt : boundary) {
                path.push_back(Point64(static_cast<int64_t>(pt.x * scale),
                                       static_cast<int64_t>(pt.y * scale)));
            }
            clipperPaths.push_back(path);
        }

        // 创建向内偏移的填充线
        for (int i = 1; i <= iterations; ++i) {
            double offset = -i * spacing * scale;  // 负值表示向内偏移

            ClipperOffset offsetter;
            offsetter.AddPaths(clipperPaths, JoinType::Round, EndType::Polygon);

            Paths64 offsetPaths;
            offsetter.Execute(offset, offsetPaths);

            // 转换回我们的格式
            for (const auto& path : offsetPaths) {
                Path convertedPath;
                for (const auto& pt : path) {
                    convertedPath.push_back(geometrycentral::Vector2{pt.x / scale, pt.y / scale});
                }
                if (!convertedPath.empty()) {
                    pattern.fillLines.push_back(convertedPath);
                }
            }
        }

        std::cout << "Created offset pattern with " << pattern.fillLines.size() << " fill lines" << std::endl;
        return pattern;
    }

    /**
     * 创建网格图案
     */
    static FillPattern createGridPattern(const Paths& boundaries, double spacing) {
        FillPattern pattern;
        pattern.outlines = boundaries;
        pattern.spacing = spacing;

        // 计算边界框
        double minX = 1e10, minY = 1e10, maxX = -1e10, maxY = -1e10;
        for (const auto& boundary : boundaries) {
            for (const auto& pt : boundary) {
                minX = std::min(minX, pt.x);
                minY = std::min(minY, pt.y);
                maxX = std::max(maxX, pt.x);
                maxY = std::max(maxY, pt.y);
            }
        }

        // 创建网格线
        // 垂直线
        for (double x = minX; x <= maxX; x += spacing) {
            Path line;
            line.push_back(geometrycentral::Vector2{x, minY});
            line.push_back(geometrycentral::Vector2{x, maxY});
            pattern.fillLines.push_back(line);
        }

        // 水平线
        for (double y = minY; y <= maxY; y += spacing) {
            Path line;
            line.push_back(geometrycentral::Vector2{minX, y});
            line.push_back(geometrycentral::Vector2{maxX, y});
            pattern.fillLines.push_back(line);
        }

        return pattern;
    }
};

/**
 * 主测试流程
 */
int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "Complete Pipeline Test" << std::endl;
    std::cout << "========================================" << std::endl;

    // 默认输入文件
    std::string inputFile = "../../../../data/spot.obj";
    if (argc > 1) {
        inputFile = argv[1];
    }

    try {
        // ========================================
        // 步骤1: 加载Spot模型
        // ========================================
        std::cout << "\n[Step 1] Loading model: " << inputFile << std::endl;

        std::unique_ptr<surface::ManifoldSurfaceMesh> mesh;
        std::unique_ptr<surface::VertexPositionGeometry> geometry;
        std::tie(mesh, geometry) = surface::readManifoldSurfaceMesh(inputFile);

        std::cout << "  Loaded mesh statistics:" << std::endl;
        std::cout << "    Vertices: " << mesh->nVertices() << std::endl;
        std::cout << "    Faces: " << mesh->nFaces() << std::endl;
        std::cout << "    Edges: " << mesh->nEdges() << std::endl;
        std::cout << "    Boundary loops: " << mesh->nBoundaryLoops() << std::endl;
        std::cout << "    Is closed: " << (mesh->nBoundaryLoops() == 0 ? "Yes" : "No") << std::endl;

        // ========================================
        // 步骤2: 网格预处理
        // ========================================
        std::cout << "\n[Step 2] Mesh preprocessing..." << std::endl;

        MeshProcessor processor;
        auto meshPtr = std::shared_ptr<surface::ManifoldSurfaceMesh>(mesh.release());
        auto geometryPtr = std::shared_ptr<surface::VertexPositionGeometry>(geometry.release());
        processor.setMesh(meshPtr, geometryPtr);

        // 计算预处理参数
        auto stats = processor.computeStats();
        std::cout << "  Original mean edge length: " << stats.meanEdgeLength << std::endl;

        // 执行等距重网格化
        MeshProcessor::ProcessingParams procParams;
        procParams.targetEdgeLength = stats.meanEdgeLength * 0.8;  // 稍微细化
        procParams.iterations = 3;
        procParams.smoothingWeight = 0.1;

        auto [remeshedMesh, remeshedGeometry] = processor.remeshIsotropic(procParams);

        if (remeshedMesh && remeshedGeometry) {
            meshPtr = remeshedMesh;
            geometryPtr = remeshedGeometry;
            std::cout << "  After remeshing: " << meshPtr->nVertices()
                      << " vertices, " << meshPtr->nFaces() << " faces" << std::endl;
        }

        // 应用平滑
        processor.setMesh(meshPtr, geometryPtr);
        processor.smoothLaplacian(2, 0.5);
        std::cout << "  Applied Laplacian smoothing" << std::endl;

        // ========================================
        // 步骤3: Variational Surface Cutting
        // ========================================
        std::cout << "\n[Step 3] Variational surface cutting..." << std::endl;

        VariationalCutter cutter;
        cutter.setMesh(meshPtr, geometryPtr);

        // 设置切割参数
        VariationalCutter::CuttingParams cutParams;
        cutParams.lengthRegularization = 0.1;
        cutParams.smoothRegularization = 0.05;
        cutParams.maxIterations = 50;
        cutParams.convergenceThreshold = 1e-4;

        // 计算最优切割
        auto cuts = cutter.computeOptimalCuts(cutParams);
        std::cout << "  Generated " << cuts.size() << " cut curves" << std::endl;

        for (size_t i = 0; i < cuts.size(); ++i) {
            std::cout << "    Cut " << i << ": " << cuts[i].points.size()
                      << " points, length = " << cuts[i].totalLength << std::endl;
        }

        // ========================================
        // 步骤4: BFF展开UV
        // ========================================
        std::cout << "\n[Step 4] BFF UV unwrapping..." << std::endl;

        // 创建纹理映射器
        TextureMapper mapper;
        mapper.setMesh(meshPtr, geometryPtr);

        // 设置映射参数
        TextureMapper::MappingParams mapParams;
        mapParams.useConformalMapping = true;
        mapParams.automaticConeDetection = true;
        mapParams.curvatureThreshold = 0.1;

        // 计算UV映射
        auto uvMappingOpt = mapper.computeUVMapping(mapParams);

        if (!uvMappingOpt.has_value()) {
            std::cerr << "Failed to compute UV mapping!" << std::endl;
            return 1;
        }

        auto& uvMapping = uvMappingOpt.value();
        std::cout << "  UV mapping successful!" << std::endl;
        std::cout << "    UV coordinates: " << uvMapping.uvCoordinates.size() << std::endl;
        std::cout << "    Charts: " << uvMapping.charts.size() << std::endl;
        std::cout << "    Total distortion: " << uvMapping.totalDistortion << std::endl;

        // 计算失真度量
        auto metrics = mapper.computeDistortionMetrics(uvMapping);
        std::cout << "  Distortion metrics:" << std::endl;
        std::cout << "    Angle distortion: " << metrics.angleDistortion << std::endl;
        std::cout << "    Area distortion: " << metrics.areaDistortion << std::endl;
        std::cout << "    Conformal error: " << metrics.conformalError << std::endl;

        // ========================================
        // 步骤5: Clipper2 Offset填充
        // ========================================
        std::cout << "\n[Step 5] Creating offset fill pattern with Clipper2..." << std::endl;

        // 提取UV边界
        std::vector<PatternFiller::Path> uvBoundaries;

        // 简单示例：创建单位正方形边界
        PatternFiller::Path squareBoundary;
        squareBoundary.push_back(geometrycentral::Vector2{0, 0});
        squareBoundary.push_back(geometrycentral::Vector2{1, 0});
        squareBoundary.push_back(geometrycentral::Vector2{1, 1});
        squareBoundary.push_back(geometrycentral::Vector2{0, 1});
        uvBoundaries.push_back(squareBoundary);

        // 创建偏移填充图案
        double spacing = 0.02;  // 线条间距
        int iterations = 20;     // 偏移迭代次数

        auto fillPattern = PatternFiller::createOffsetPattern(uvBoundaries, spacing, iterations);
        std::cout << "  Created offset pattern with " << fillPattern.fillLines.size() << " lines" << std::endl;

        // 也可以创建网格图案
        auto gridPattern = PatternFiller::createGridPattern(uvBoundaries, spacing);
        std::cout << "  Created grid pattern with " << gridPattern.fillLines.size() << " lines" << std::endl;

        // ========================================
        // 步骤6: 纹理映射到原模型
        // ========================================
        std::cout << "\n[Step 6] Mapping texture back to original model..." << std::endl;

        // 创建纹理坐标数组
        std::vector<geometrycentral::Vector2> textureCoords(meshPtr->nVertices());

        // 将UV坐标分配给顶点
        for (size_t i = 0; i < uvMapping.uvCoordinates.size() && i < textureCoords.size(); ++i) {
            textureCoords[i] = uvMapping.uvCoordinates[i];
        }

        // 导出带UV的网格
        std::string outputFile = "spot_with_uv.obj";

        // 手动创建OBJ文件
        std::ofstream objFile(outputFile);
        if (objFile.is_open()) {
            // 写入顶点
            for (auto v : meshPtr->vertices()) {
                auto pos = geometryPtr->vertexPositions[v];
                objFile << "v " << pos.x << " " << pos.y << " " << pos.z << "\n";
            }

            // 写入UV坐标
            for (const auto& uv : textureCoords) {
                objFile << "vt " << uv.x << " " << uv.y << "\n";
            }

            // 写入面（带UV索引）
            for (auto f : meshPtr->faces()) {
                objFile << "f";
                for (auto v : f.adjacentVertices()) {
                    int idx = v.getIndex() + 1;  // OBJ索引从1开始
                    objFile << " " << idx << "/" << idx;
                }
                objFile << "\n";
            }

            objFile.close();
            std::cout << "  Exported mesh with UV to: " << outputFile << std::endl;
        }

        // ========================================
        // 完成
        // ========================================
        std::cout << "\n========================================" << std::endl;
        std::cout << "Pipeline completed successfully!" << std::endl;
        std::cout << "========================================" << std::endl;

        std::cout << "\nSummary:" << std::endl;
        std::cout << "  1. Loaded model: " << inputFile << std::endl;
        std::cout << "  2. Preprocessed mesh (remeshing + smoothing)" << std::endl;
        std::cout << "  3. Computed " << cuts.size() << " variational cuts" << std::endl;
        std::cout << "  4. UV unwrapped with BFF (distortion: " << uvMapping.totalDistortion << ")" << std::endl;
        std::cout << "  5. Created offset fill pattern (" << fillPattern.fillLines.size() << " lines)" << std::endl;
        std::cout << "  6. Exported result to: " << outputFile << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}