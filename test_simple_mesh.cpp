/**
 * 最简单的网格转换测试
 * 逐步调试崩溃问题
 */

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/meshio.h"

// Core几何库
#include "halfedge_mesh.h"
#include "geometry.h"
#include "polygon_soup_mesh.h"
#include "vector3.h"

#include <iostream>
#include <memory>

using namespace geometrycentral;
using namespace geometrycentral::surface;

int main() {
    std::cout << "Loading mesh..." << std::endl;
    std::cout.flush();
    
    std::unique_ptr<ManifoldSurfaceMesh> mesh;
    std::unique_ptr<VertexPositionGeometry> geometry;
    std::tie(mesh, geometry) = readManifoldSurfaceMesh("data/spot.obj");

    std::cout << "Mesh loaded: " << mesh->nVertices() << " vertices" << std::endl;
    std::cout.flush();

    // 提取顶点位置
    std::vector<::Vector3> vertexPositions;  // Core库的Vector3
    vertexPositions.reserve(mesh->nVertices());
    for (auto v : mesh->vertices()) {
        auto pos = geometry->vertexPositions[v];
        vertexPositions.push_back(::Vector3{pos.x, pos.y, pos.z});
    }
    std::cout << "Extracted " << vertexPositions.size() << " vertex positions" << std::endl;
    std::cout.flush();

    // 提取面信息
    std::vector<std::vector<size_t>> faces;
    faces.reserve(mesh->nFaces());
    for (auto f : mesh->faces()) {
        std::vector<size_t> faceIndices;
        for (auto v : f.adjacentVertices()) {
            faceIndices.push_back(v.getIndex());
        }
        faces.push_back(faceIndices);
    }
    std::cout << "Extracted " << faces.size() << " faces" << std::endl;
    std::cout.flush();

    // 创建PolygonSoupMesh
    std::cout << "Creating PolygonSoupMesh..." << std::endl;
    std::cout.flush();
    PolygonSoupMesh soupMesh(faces, vertexPositions);
    std::cout << "PolygonSoupMesh created" << std::endl;
    std::cout.flush();

    // 创建HalfedgeMesh
    std::cout << "Creating HalfedgeMesh..." << std::endl;
    std::cout.flush();
    Geometry<Euclidean>* tempGeometry = nullptr;
    ::HalfedgeMesh* tempMesh = new ::HalfedgeMesh(soupMesh, tempGeometry);
    std::cout << "HalfedgeMesh created!" << std::endl;
    std::cout.flush();

    // 测试访问方法
    std::cout << "Testing nVertices()..." << std::endl;
    std::cout.flush();
    size_t nV = tempMesh->nVertices();
    std::cout << "nVertices() = " << nV << std::endl;
    std::cout.flush();

    std::cout << "Testing nFaces()..." << std::endl;
    std::cout.flush();
    size_t nF = tempMesh->nFaces();
    std::cout << "nFaces() = " << nF << std::endl;
    std::cout.flush();

    std::cout << "Testing nEdges()..." << std::endl;
    std::cout.flush();
    size_t nE = tempMesh->nEdges();
    std::cout << "nEdges() = " << nE << std::endl;
    std::cout.flush();

    std::cout << "All tests passed!" << std::endl;

    delete tempMesh;

    return 0;
}
