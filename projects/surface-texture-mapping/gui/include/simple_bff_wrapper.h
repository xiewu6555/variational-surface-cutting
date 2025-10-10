#pragma once

#include <memory>
#include <vector>
#include <geometry-central/surface/halfedge_mesh.h>
#include <geometry-central/surface/vertex_position_geometry.h>
#include <geometry-central/utilities/vector2.h>

namespace SurfaceTextureMapping {

using namespace geometrycentral;
using namespace geometrycentral::surface;

// 简单的BFF包装器 - 使用基于热方法的简化参数化
class SimpleBFFWrapper {
public:
    SimpleBFFWrapper() = default;
    ~SimpleBFFWrapper() = default;

    bool setMesh(HalfedgeMesh* mesh, VertexPositionGeometry* geometry);

    struct UVResult {
        bool success;
        std::vector<Vector2> vertexUVs;
        std::string errorMessage;
    };

    UVResult computeParameterization();

private:
    HalfedgeMesh* mesh_ = nullptr;
    VertexPositionGeometry* geometry_ = nullptr;
};

} // namespace SurfaceTextureMapping