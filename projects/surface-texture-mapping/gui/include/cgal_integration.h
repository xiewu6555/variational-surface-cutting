#pragma once

// CGAL网格处理集成头文件
// 定义CGAL类型和辅助函数

#include <vector>
#include <array>
#include <map>

// TODO: CGAL集成暂时禁用，使用简化的类型定义
// 原因：CGAL依赖复杂，需要通过vcpkg或手动安装
// 待后续安装CGAL后再启用

// CGAL已通过vcpkg安装完成，但暂时禁用以简化编译
// #define USE_CGAL

// 确保USE_CGAL未定义，避免编译错误
#ifdef USE_CGAL
#undef USE_CGAL
#endif

// 仅当明确定义了ENABLE_CGAL时才包含CGAL头文件
#ifdef ENABLE_CGAL
// CGAL核心类型
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/repair.h>
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/measure.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

namespace SurfaceTextureMapping {

// CGAL类型定义
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3> SurfaceMesh;
typedef SurfaceMesh::Vertex_index vertex_descriptor;
typedef SurfaceMesh::Face_index face_descriptor;
typedef SurfaceMesh::Edge_index edge_descriptor;
typedef SurfaceMesh::Halfedge_index halfedge_descriptor;
typedef K::Point_3 Point_3;

#else // 简化的类型定义（不使用CGAL）

namespace SurfaceTextureMapping {

// 简化的网格类型（用于代替CGAL Surface_mesh）
struct SurfaceMesh {
    std::vector<std::array<float, 3>> vertices;
    std::vector<std::array<int, 3>> faces;
};

typedef int vertex_descriptor;
typedef int face_descriptor;
typedef int edge_descriptor;
typedef std::array<float, 3> Point_3;

#endif // ENABLE_CGAL

// 网格质量统计结构（移到这里）
struct MeshQualityStats {
    double min_edge_length = 0.0;
    double max_edge_length = 0.0;
    double avg_edge_length = 0.0;
    double edge_length_std_dev = 0.0;
    int degenerate_faces = 0;
    int self_intersections = 0;
    bool is_manifold = false;
    bool is_closed = false;
    bool is_oriented = false;
    double surface_area = 0.0;
    double volume = 0.0;
    int genus = 0;
    int boundary_components = 0;
};

// CGAL网格处理函数声明（在cgal_mesh_processing.cpp中实现）
MeshQualityStats analyzeMeshQualityCGAL(const SurfaceMesh& cgal_mesh);
bool repairMeshCGAL(SurfaceMesh& cgal_mesh);
bool ensureProperOrientationCGAL(SurfaceMesh& cgal_mesh);
bool removeSelfIntersectionsCGAL(SurfaceMesh& cgal_mesh);
bool performIsotropicRemeshingCGAL(SurfaceMesh& cgal_mesh, double target_edge_length = 0.01, int iterations = 3);

} // namespace SurfaceTextureMapping