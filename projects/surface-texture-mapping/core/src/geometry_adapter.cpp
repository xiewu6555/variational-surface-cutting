/**
 * 几何适配层实现
 * 完整的geometry-central <--> core几何库转换
 */

#include "geometry_adapter.h"

// Core几何库 - 使用完整路径避免冲突
#include "halfedge_mesh.h"
#include "geometry.h"
#include "polygon_soup_mesh.h"
#include "vector3.h"

// geometry-central网格构造
#include "geometrycentral/surface/simple_polygon_mesh.h"
#include "geometrycentral/surface/meshio.h"

#include <iostream>
#include <cmath>
#include <algorithm>
#include <tuple>

namespace SurfaceTextureMapping {

/**
 * geometry-central --> core 转换实现
 */
GeometryAdapter::ConversionResult GeometryAdapter::convertFromGeometryCentral(
    std::shared_ptr<GeometryTypeMapping::GC_Mesh> gcMesh,
    std::shared_ptr<GeometryTypeMapping::GC_Geometry> gcGeometry,
    GeometryTypeMapping::Core_Mesh*& outMesh,
    GeometryTypeMapping::Core_Geometry*& outGeometry,
    const ConversionOptions& options) {

    ConversionResult result;
    result.success = false;

    if (!gcMesh || !gcGeometry) {
        result.errorMessage = "输入网格或几何为空";
        return result;
    }

    if (options.verbose) {
        std::cout << "=== Geometry-Central to Core Conversion ===" << std::endl;
        std::cout << "  Input: " << gcMesh->nVertices() << " vertices, "
                  << gcMesh->nFaces() << " faces, "
                  << gcMesh->nEdges() << " edges" << std::endl;
    }

    try {
        // Step 1: 提取顶点位置
        std::vector<Vector3> vertexPositions;
        vertexPositions.reserve(gcMesh->nVertices());

        for (auto v : gcMesh->vertices()) {
            auto pos = gcGeometry->vertexPositions[v];
            // 转换geometry-central::Vector3 --> core::Vector3
            vertexPositions.push_back(Vector3{pos.x, pos.y, pos.z});
        }

        // Step 2: 提取面信息（保持顶点顺序）
        std::vector<std::vector<size_t>> faces;
        faces.reserve(gcMesh->nFaces());

        for (auto f : gcMesh->faces()) {
            std::vector<size_t> faceIndices;
            for (auto v : f.adjacentVertices()) {
                faceIndices.push_back(v.getIndex());
            }
            faces.push_back(faceIndices);
        }

        // Step 3: 创建PolygonSoupMesh作为中间格式
        PolygonSoupMesh soupMesh(faces, vertexPositions);

        // Step 4: 转换为HalfedgeMesh
        // 注意：HalfedgeMesh构造函数会自动创建Geometry对象
        // 必须使用Euclidean而不是Vector3，因为EulerianShapeOptimizer需要Geometry<Euclidean>*
        if (options.verbose) std::cout << "  Creating HalfedgeMesh..." << std::endl;
        Geometry<Euclidean>* tempGeometry = nullptr;
        ::HalfedgeMesh* tempMesh = new ::HalfedgeMesh(soupMesh, tempGeometry);
        if (options.verbose) std::cout << "  HalfedgeMesh created" << std::endl;
        outMesh = tempMesh;  // 转换为void*
        outGeometry = tempGeometry;

        // Step 5: 填充统计信息
        if (options.verbose) std::cout << "  Computing statistics..." << std::endl;
        result.numVertices = tempMesh->nVertices();
        result.numFaces = tempMesh->nFaces();
        result.numEdges = tempMesh->nEdges();

        // Step 6: 验证转换（如果启用）
        if (options.validateTopology) {
            if (!validateTopology(gcMesh, outMesh)) {
                throw std::runtime_error("拓扑验证失败");
            }
        }

        // Step 7: 计算几何误差
        if (options.preserveAttributes) {
            computeGeometricErrors(gcGeometry, outGeometry, result);
        }

        result.success = true;

        if (options.verbose) {
            std::cout << "  Output: " << result.numVertices << " vertices, "
                      << result.numFaces << " faces, "
                      << result.numEdges << " edges" << std::endl;
            std::cout << "  Max position error: " << result.maxPositionError << std::endl;
            std::cout << "  Max edge length error: " << result.maxEdgeLengthError << std::endl;
            std::cout << "  Conversion SUCCESS!" << std::endl;
        }

    } catch (const std::exception& e) {
        result.errorMessage = std::string("转换失败: ") + e.what();
        result.success = false;

        // 清理已分配的内存
        if (outMesh) {
            delete outMesh;
            outMesh = nullptr;
        }
        outGeometry = nullptr;  // 由mesh管理

        if (options.verbose) {
            std::cerr << "  Conversion FAILED: " << result.errorMessage << std::endl;
        }
    }

    return result;
}

/**
 * core --> geometry-central 转换实现
 * TODO: 需要找到geometry-central正确的网格构造API
 */
GeometryAdapter::ConversionResult GeometryAdapter::convertToGeometryCentral(
    const GeometryTypeMapping::Core_Mesh* coreMesh,
    const GeometryTypeMapping::Core_Geometry* coreGeometry,
    std::shared_ptr<GeometryTypeMapping::GC_Mesh>& outMesh,
    std::shared_ptr<GeometryTypeMapping::GC_Geometry>& outGeometry,
    const ConversionOptions& options) {

    ConversionResult result;
    result.success = false;
    result.errorMessage = "convertToGeometryCentral: Not implemented yet";

    // 暂时不支持反向转换，因为geometry-central的API需要进一步研究
    (void)coreMesh;
    (void)coreGeometry;
    (void)outMesh;
    (void)outGeometry;
    (void)options;

    return result;
}

/**
 * 验证转换质量
 */
GeometryAdapter::ConversionResult GeometryAdapter::validateConversion(
    std::shared_ptr<GeometryTypeMapping::GC_Mesh> originalMesh,
    std::shared_ptr<GeometryTypeMapping::GC_Geometry> originalGeometry,
    const GeometryTypeMapping::Core_Mesh* convertedMesh,
    const GeometryTypeMapping::Core_Geometry* convertedGeometry) {

    ConversionResult result;
    result.success = true;

    // 转换void*到具体类型
    auto* coreM = static_cast<const ::HalfedgeMesh*>(convertedMesh);
    auto* coreG = static_cast<const Geometry<Euclidean>*>(convertedGeometry);

    // 检查基本拓扑
    bool vertexCountMatch = (originalMesh->nVertices() == coreM->nVertices());
    bool faceCountMatch = (originalMesh->nFaces() == coreM->nFaces());
    bool edgeCountMatch = (originalMesh->nEdges() == coreM->nEdges());

    if (!vertexCountMatch || !faceCountMatch || !edgeCountMatch) {
        result.success = false;
        result.errorMessage = "拓扑不匹配: ";
        if (!vertexCountMatch) result.errorMessage += "顶点数; ";
        if (!faceCountMatch) result.errorMessage += "面数; ";
        if (!edgeCountMatch) result.errorMessage += "边数; ";
    }

    // 计算几何误差
    if (result.success) {
        computeGeometricErrors(originalGeometry, convertedGeometry, result);

        // 设置误差阈值
        const double positionTolerance = 1e-6;
        const double lengthTolerance = 1e-6;

        if (result.maxPositionError > positionTolerance ||
            result.maxEdgeLengthError > lengthTolerance) {
            result.success = false;
            result.errorMessage = "几何误差超出容限";
        }
    }

    return result;
}

/**
 * 验证拓扑等价
 */
bool GeometryAdapter::validateTopology(
    std::shared_ptr<GeometryTypeMapping::GC_Mesh> mesh1,
    const GeometryTypeMapping::Core_Mesh* mesh2) {

    // 转换void*到具体类型
    auto* coreM = static_cast<const ::HalfedgeMesh*>(mesh2);

    // 基本统计检查
    if (mesh1->nVertices() != coreM->nVertices()) return false;
    if (mesh1->nFaces() != coreM->nFaces()) return false;
    if (mesh1->nEdges() != coreM->nEdges()) return false;

    // TODO: 可以添加更深入的拓扑检查
    // 例如：检查每个面的邻接关系等

    return true;
}

/**
 * 计算几何误差
 */
void GeometryAdapter::computeGeometricErrors(
    std::shared_ptr<GeometryTypeMapping::GC_Geometry> geom1,
    const GeometryTypeMapping::Core_Geometry* geom2,
    ConversionResult& result) {

    // 转换void*到具体类型 (去掉const，因为Geometry的方法不是const的)
    auto* coreG = const_cast<Geometry<Euclidean>*>(static_cast<const Geometry<Euclidean>*>(geom2));

    // 计算顶点位置误差
    result.maxPositionError = 0.0;
    size_t vertIdx = 0;
    for (auto v : geom1->mesh.vertices()) {
        auto pos1 = geom1->vertexPositions[v];

        // 获取core网格对应顶点
        // 假设顶点索引对应
        VertexPtr v2 = coreG->mesh.vertex(vertIdx);
        Vector3 pos2 = coreG->position(v2);

        double error = std::sqrt(
            (pos1.x - pos2.x) * (pos1.x - pos2.x) +
            (pos1.y - pos2.y) * (pos1.y - pos2.y) +
            (pos1.z - pos2.z) * (pos1.z - pos2.z)
        );

        result.maxPositionError = std::max(result.maxPositionError, error);
        vertIdx++;
    }

    // 计算边长误差
    result.maxEdgeLengthError = 0.0;
    size_t edgeIdx = 0;
    for (auto e : geom1->mesh.edges()) {
        double len1 = geom1->edgeLengths[e];

        EdgePtr e2 = coreG->mesh.edge(edgeIdx);
        double len2 = coreG->length(e2);

        double error = std::abs(len1 - len2);
        result.maxEdgeLengthError = std::max(result.maxEdgeLengthError, error);
        edgeIdx++;
    }

    // 计算面积误差
    result.maxAreaError = 0.0;
    size_t faceIdx = 0;
    for (auto f : geom1->mesh.faces()) {
        double area1 = geom1->faceAreas[f];

        FacePtr f2 = coreG->mesh.face(faceIdx);
        double area2 = coreG->area(f2);

        double error = std::abs(area1 - area2);
        result.maxAreaError = std::max(result.maxAreaError, error);
        faceIdx++;
    }
}

} // namespace SurfaceTextureMapping
