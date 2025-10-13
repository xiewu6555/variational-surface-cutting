/**
 * 测试网格转换的独立程序
 * 用于重现和验证崩溃修复
 */

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

#include "geometry_adapter.h"

#include <iostream>
#include <memory>

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace SurfaceTextureMapping;

int main(int argc, char** argv) {
    std::string inputMesh = "data/spot.obj";

    if (argc > 1) {
        inputMesh = argv[1];
    }

    std::cout << "Testing mesh conversion with: " << inputMesh << std::endl;

    try {
        // 加载网格
        std::cout << "Loading mesh..." << std::endl;
        std::unique_ptr<ManifoldSurfaceMesh> mesh;
        std::unique_ptr<VertexPositionGeometry> geometry;
        std::tie(mesh, geometry) = readManifoldSurfaceMesh(inputMesh);

        std::cout << "Mesh loaded: " << mesh->nVertices() << " vertices, "
                  << mesh->nFaces() << " faces, "
                  << mesh->nEdges() << " edges" << std::endl;

        // 转换为Core格式
        std::cout << "\nConverting to Core format..." << std::endl;

        GeometryTypeMapping::Core_Mesh* coreMesh = nullptr;
        GeometryTypeMapping::Core_Geometry* coreGeometry = nullptr;

        GeometryAdapter::ConversionOptions options;
        options.verbose = true;
        options.validateTopology = true;
        options.preserveAttributes = true;

        auto meshShared = std::shared_ptr<ManifoldSurfaceMesh>(mesh.get(), [](ManifoldSurfaceMesh*){});
        auto geomShared = std::shared_ptr<VertexPositionGeometry>(geometry.get(), [](VertexPositionGeometry*){});

        auto result = GeometryAdapter::convertFromGeometryCentral(
            meshShared, geomShared,
            coreMesh, coreGeometry,
            options
        );

        if (result.success) {
            std::cout << "\n=== Conversion SUCCESS ===" << std::endl;
            std::cout << "Output: " << result.numVertices << " vertices, "
                      << result.numFaces << " faces, "
                      << result.numEdges << " edges" << std::endl;
            std::cout << "Position error: " << result.maxPositionError << std::endl;
            std::cout << "Edge length error: " << result.maxEdgeLengthError << std::endl;
            std::cout << "Area error: " << result.maxAreaError << std::endl;

            // 清理 (使用CoreMeshWrapper自动管理)
            SurfaceTextureMapping::CoreMeshWrapper wrapper;
            wrapper.reset(coreMesh, coreGeometry);

            return 0;
        } else {
            std::cerr << "\n=== Conversion FAILED ===" << std::endl;
            std::cerr << "Error: " << result.errorMessage << std::endl;
            return 1;
        }

    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
}
