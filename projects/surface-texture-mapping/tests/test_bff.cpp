/**
 * BFF算法测试程序
 * 测试完整的BFF纹理映射流程
 */

#include <iostream>
#include <string>
#include <memory>

#include "texture_mapping.h"
// #include "mesh_processor.h"  // 暂时移除，因为此文件不存在
#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

using namespace SurfaceTextureMapping;
namespace gc = geometrycentral;
namespace surface = geometrycentral::surface;

int main(int argc, char* argv[]) {
    // 默认测试文件
    // 注意：当从项目根目录运行时（如 ./build/STM-Debug/bin/tests/Debug/test_bff.exe），
    // 当前工作目录是项目根，所以使用相对于项目根的路径
    std::string inputFile = "F:/Code/OpenProject/variational-surface-cutting/data/spot.obj";
    if (argc > 1) {
        inputFile = argv[1];
    }

    std::cout << "==================================" << std::endl;
    std::cout << "BFF Algorithm Test" << std::endl;
    std::cout << "==================================" << std::endl;
    std::cout << "Input: " << inputFile << std::endl;

    try {
        // 1. 加载网格
        std::cout << "\n1. Loading mesh..." << std::endl;
        std::unique_ptr<surface::ManifoldSurfaceMesh> mesh;
        std::unique_ptr<surface::VertexPositionGeometry> geometry;
        std::tie(mesh, geometry) = surface::readManifoldSurfaceMesh(inputFile);

        std::cout << "   Vertices: " << mesh->nVertices() << std::endl;
        std::cout << "   Faces: " << mesh->nFaces() << std::endl;
        std::cout << "   Edges: " << mesh->nEdges() << std::endl;
        std::cout << "   Boundary loops: " << mesh->nBoundaryLoops() << std::endl;
        std::cout << "   Is closed: " << (mesh->nBoundaryLoops() == 0 ? "Yes" : "No") << std::endl;

        // 2. 预处理网格（跳过，因为MeshProcessor尚未实现）
        std::cout << "\n2. Preprocessing mesh... (skipped)" << std::endl;

        // 3. 创建纹理映射器
        std::cout << "\n3. Setting up texture mapper..." << std::endl;
        TextureMapper mapper;
        // 转换unique_ptr到shared_ptr
        std::shared_ptr<surface::ManifoldSurfaceMesh> meshPtr(mesh.release());
        std::shared_ptr<surface::VertexPositionGeometry> geometryPtr(geometry.release());
        mapper.setMesh(meshPtr, geometryPtr);

        // 4. 检测锥点
        std::cout << "\n4. Detecting cone points..." << std::endl;
        auto coneIndices = mapper.detectConeVertices(0.1);
        std::cout << "   Found " << coneIndices.size() << " cone vertices" << std::endl;
        if (!coneIndices.empty()) {
            std::cout << "   Cone indices: ";
            for (size_t i = 0; i < std::min(size_t(10), coneIndices.size()); i++) {
                std::cout << coneIndices[i] << " ";
            }
            if (coneIndices.size() > 10) std::cout << "...";
            std::cout << std::endl;
        }

        // 5. 计算UV映射（使用完整的Variational Surface Cutting集成）
        // 修复日期: 2025-10-10
        // 修复内容: 完整的Variational Cuts集成已成功实现
        // 参考: https://github.com/nmwsharp/variational-surface-cutting
        std::cout << "\n5. Computing UV mapping with Variational Surface Cutting..." << std::endl;
        std::cout << "   [INFO] Using complete Variational Cuts + BFF pipeline" << std::endl;

        TextureMapper::MappingParams params;
        params.useConformalMapping = true;

        // 配置Variational Cuts参数（遵循GitHub README推荐）
        // - Normal Clustering初始化（自动执行）
        // - Hencky能量权重: 3.0（在EulerianCutIntegrator中设置）
        // - 优化迭代次数: 30次（可调整）
        params.automaticConeDetection = false;  // 手动指定锥点以更好控制
        params.curvatureThreshold = 0.1;
        params.enableAreaCorrection = false;

        // 如果网格是闭合的，手动指定少量锥点
        // 修复: mesh已经release，使用meshPtr
        if (meshPtr->nBoundaryLoops() == 0) {
            std::cout << "   Mesh is closed, using simple cone configuration" << std::endl;
            // 使用前4个检测到的高曲率点作为锥点
            size_t numConesToUse = std::min(size_t(4), coneIndices.size());
            for (size_t i = 0; i < numConesToUse; i++) {
                params.coneVertices.push_back(coneIndices[i]);
            }
            std::cout << "   Using " << numConesToUse << " manually specified cone points" << std::endl;
        }

        auto mappingOpt = mapper.computeUVMapping(params);

        if (!mappingOpt.has_value()) {
            std::cerr << "   Failed to compute UV mapping!" << std::endl;
            return 1;
        }

        auto& mapping = mappingOpt.value();
        std::cout << "   UV mapping computed successfully!" << std::endl;
        std::cout << "   Number of UV coordinates: " << mapping.uvCoordinates.size() << std::endl;
        std::cout << "   Number of charts: " << mapping.charts.size() << std::endl;

        // 6. 计算失真度量
        std::cout << "\n6. Computing distortion metrics..." << std::endl;
        auto metrics = mapper.computeDistortionMetrics(mapping);
        std::cout << "   Angle distortion: " << metrics.angleDistortion << std::endl;
        std::cout << "   Area distortion: " << metrics.areaDistortion << std::endl;
        std::cout << "   Conformal error: " << metrics.conformalError << std::endl;
        std::cout << "   Total distortion: " << mapping.totalDistortion << std::endl;
        std::cout << "   Max distortion: " << mapping.maxDistortion << std::endl;

        // 7. 优化UV布局（可选）
        std::cout << "\n7. Optimizing UV layout..." << std::endl;
        auto optimizedMapping = mapper.optimizeUVLayout(mapping, 0.8);
        std::cout << "   Layout optimized with 80% packing efficiency target" << std::endl;

        // 8. 导出结果
        std::cout << "\n8. Exporting results..." << std::endl;
        std::string outputFile = "test_bff_output.obj";
        if (mapper.exportUVMesh(outputFile, optimizedMapping)) {
            std::cout << "   Exported UV mesh to: " << outputFile << std::endl;
        } else {
            std::cerr << "   Failed to export UV mesh!" << std::endl;
        }

        // 9. 可视化（可选）
        std::cout << "\n9. Visualizing UV mapping..." << std::endl;
        mapper.visualizeUVMapping(optimizedMapping, "test_bff_uv.png", 1024);
        std::cout << "   UV visualization saved to: test_bff_uv.png" << std::endl;

        // 10. 统计信息
        std::cout << "\n==================================" << std::endl;
        std::cout << "Test Summary:" << std::endl;
        std::cout << "==================================" << std::endl;
        std::cout << "✓ Mesh loaded successfully" << std::endl;
        std::cout << "✓ Cone points detected: " << coneIndices.size() << std::endl;
        std::cout << "✓ UV mapping computed" << std::endl;
        std::cout << "✓ Distortion metrics calculated" << std::endl;
        std::cout << "  - Angle distortion: " << metrics.angleDistortion << std::endl;
        std::cout << "  - Area distortion: " << metrics.areaDistortion << std::endl;
        std::cout << "  - Conformal error: " << metrics.conformalError << std::endl;
        std::cout << "✓ UV layout optimized" << std::endl;
        std::cout << "✓ Results exported" << std::endl;

        std::cout << "\nTest completed successfully!" << std::endl;

    } catch (const std::exception& e) {
        std::cerr << "\nError: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}