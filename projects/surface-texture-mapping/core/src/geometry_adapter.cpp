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
#include <unordered_map>
#include <limits>

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
        if (options.verbose) {
            std::cout << "  Creating HalfedgeMesh..." << std::endl;
            std::cout.flush();
        }
        Geometry<Euclidean>* tempGeometry = nullptr;
        ::HalfedgeMesh* tempMesh = new ::HalfedgeMesh(soupMesh, tempGeometry);
        if (options.verbose) {
            std::cout << "  HalfedgeMesh created" << std::endl;
            std::cout.flush();
        }
        outMesh = tempMesh;  // 转换为void*
        outGeometry = tempGeometry;

        // Step 4.5: 构建GC与core顶点索引映射
        struct QuantKey {
            long long x;
            long long y;
            long long z;

            bool operator==(const QuantKey& other) const noexcept {
                return x == other.x && y == other.y && z == other.z;
            }
        };
        struct QuantKeyHash {
            size_t operator()(const QuantKey& key) const noexcept {
                size_t h = std::hash<long long>{}(key.x);
                h ^= std::hash<long long>{}(key.y) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
                h ^= std::hash<long long>{}(key.z) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
                return h;
            }
        };

        const double quantScale = 1e9;
        auto quantizeCoord = [quantScale](double value) -> long long {
            return static_cast<long long>(std::llround(value * quantScale));
        };

        using IndexBucket = std::vector<size_t>;
        std::unordered_map<QuantKey, IndexBucket, QuantKeyHash> gcPositionBuckets;
        gcPositionBuckets.reserve(vertexPositions.size());

        for (size_t idx = 0; idx < vertexPositions.size(); ++idx) {
            const Vector3& pos = vertexPositions[idx];
            QuantKey key{quantizeCoord(pos.x), quantizeCoord(pos.y), quantizeCoord(pos.z)};
            gcPositionBuckets[key].push_back(idx);
        }

        const size_t invalidIndex = std::numeric_limits<size_t>::max();
        result.coreToGCVertexIndex.assign(tempMesh->nVertices(), invalidIndex);
        result.gcToCoreVertexIndex.assign(vertexPositions.size(), invalidIndex);

        size_t coreIdx = 0;
        for (VertexPtr v : tempMesh->vertices()) {
            Vector3 corePos = tempGeometry->position(v);
            QuantKey key{quantizeCoord(corePos.x), quantizeCoord(corePos.y), quantizeCoord(corePos.z)};

            size_t gcIndex = invalidIndex;
            auto it = gcPositionBuckets.find(key);
            if (it != gcPositionBuckets.end() && !it->second.empty()) {
                gcIndex = it->second.back();
                it->second.pop_back();
            } else {
                double bestDist = std::numeric_limits<double>::max();
                for (size_t candidate = 0; candidate < vertexPositions.size(); ++candidate) {
                    const Vector3& candidatePos = vertexPositions[candidate];
                    double dx = candidatePos.x - corePos.x;
                    double dy = candidatePos.y - corePos.y;
                    double dz = candidatePos.z - corePos.z;
                    double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                    if (dist < bestDist) {
                        bestDist = dist;
                        gcIndex = candidate;
                    }
                }
                if (gcIndex == invalidIndex && options.verbose) {
                    std::cerr << "  [WARNING] Failed to match core vertex to GC vertex during mapping." << std::endl;
                }
            }

            if (gcIndex != invalidIndex) {
                result.coreToGCVertexIndex[coreIdx] = gcIndex;
                if (result.gcToCoreVertexIndex[gcIndex] == invalidIndex) {
                    result.gcToCoreVertexIndex[gcIndex] = coreIdx;
                } else if (options.verbose) {
                    std::cerr << "  [WARNING] GC vertex " << gcIndex << " already mapped; duplicate assignment detected." << std::endl;
                }
            }
            coreIdx++;
        }

        bool mappingComplete = std::all_of(
            result.coreToGCVertexIndex.begin(),
            result.coreToGCVertexIndex.end(),
            [invalidIndex](size_t value) { return value != invalidIndex; }
        );

        if (!mappingComplete && options.verbose) {
            std::cerr << "  [WARNING] Incomplete GC↔core vertex mapping detected; results may need tolerance tuning." << std::endl;
        }

        // Step 5: 填充统计信息
        if (options.verbose) {
            std::cout << "  Computing statistics..." << std::endl;
            std::cout.flush();
        }
        if (options.verbose) {
            std::cout << "    Getting nVertices..." << std::endl;
            std::cout.flush();
        }
        result.numVertices = tempMesh->nVertices();
        if (options.verbose) {
            std::cout << "    Got " << result.numVertices << " vertices" << std::endl;
            std::cout << "    Getting nFaces..." << std::endl;
            std::cout.flush();
        }
        result.numFaces = tempMesh->nFaces();
        if (options.verbose) {
            std::cout << "    Got " << result.numFaces << " faces" << std::endl;
            std::cout << "    Getting nEdges..." << std::endl;
            std::cout.flush();
        }
        result.numEdges = tempMesh->nEdges();
        if (options.verbose) {
            std::cout << "    Got " << result.numEdges << " edges" << std::endl;
            std::cout.flush();
        }

        // Step 6: 验证转换（如果启用）
        if (options.verbose) {
            std::cout << "  [DEBUG] About to validate topology (validateTopology=" << options.validateTopology << ")..." << std::endl;
            std::cout.flush();
        }
        if (options.validateTopology) {
            if (options.verbose) {
                std::cout << "  [DEBUG] Calling validateTopology()..." << std::endl;
                std::cout.flush();
            }
            if (!validateTopology(gcMesh, outMesh)) {
                throw std::runtime_error("拓扑验证失败");
            }
            if (options.verbose) {
                std::cout << "  [DEBUG] validateTopology() completed" << std::endl;
                std::cout.flush();
            }
        }

        // Step 7: 计算几何误差
        if (options.verbose) {
            std::cout << "  [DEBUG] About to compute geometric errors (preserveAttributes=" << options.preserveAttributes << ")..." << std::endl;
            std::cout.flush();
        }
        if (options.preserveAttributes) {
            if (options.verbose) {
                std::cout << "  [DEBUG] Calling computeGeometricErrors()..." << std::endl;
                std::cout.flush();
            }
            computeGeometricErrors(gcMesh, gcGeometry, outGeometry, result);
            if (options.verbose) {
                std::cout << "  [DEBUG] computeGeometricErrors() completed" << std::endl;
                std::cout.flush();
            }
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
        computeGeometricErrors(originalMesh, originalGeometry, convertedGeometry, result);

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

    std::cout << "    [DEBUG] Inside validateTopology()..." << std::endl;
    std::cout.flush();

    // 转换void*到具体类型
    std::cout << "    [DEBUG] Converting mesh pointer..." << std::endl;
    std::cout.flush();
    auto* coreM = static_cast<const ::HalfedgeMesh*>(mesh2);

    // 基本统计检查
    std::cout << "    [DEBUG] Checking nVertices..." << std::endl;
    std::cout.flush();
    if (mesh1->nVertices() != coreM->nVertices()) return false;

    std::cout << "    [DEBUG] Checking nFaces..." << std::endl;
    std::cout.flush();
    if (mesh1->nFaces() != coreM->nFaces()) return false;

    std::cout << "    [DEBUG] Checking nEdges..." << std::endl;
    std::cout.flush();
    if (mesh1->nEdges() != coreM->nEdges()) return false;

    std::cout << "    [DEBUG] Topology validation passed" << std::endl;
    std::cout.flush();

    // TODO: 可以添加更深入的拓扑检查
    // 例如：检查每个面的邻接关系等

    return true;
}

/**
 * 计算几何误差
 */
void GeometryAdapter::computeGeometricErrors(
    std::shared_ptr<GeometryTypeMapping::GC_Mesh> mesh1,
    std::shared_ptr<GeometryTypeMapping::GC_Geometry> geom1,
    const GeometryTypeMapping::Core_Geometry* geom2,
    ConversionResult& result) {

    std::cout << "    [DEBUG] Inside computeGeometricErrors()..." << std::endl;
    std::cout.flush();

    // 转换void*到具体类型 (去掉const，因为Geometry的方法不是const的)
    std::cout << "    [DEBUG] Converting geometry pointer..." << std::endl;
    std::cout.flush();
    auto* coreG = const_cast<Geometry<Euclidean>*>(static_cast<const Geometry<Euclidean>*>(geom2));

    // 关键修复：需要获取mesh的指针，而不是直接访问引用
    std::cout << "    [DEBUG] Getting mesh pointer from geometry..." << std::endl;
    std::cout.flush();
    HalfedgeMesh* coreMesh = coreG->getMesh();

    std::cout << "    [DEBUG] Checking if mesh pointer is null..." << std::endl;
    std::cout.flush();
    if (!coreMesh) {
        std::cerr << "WARNING: Core mesh pointer is null, skipping geometric error computation" << std::endl;
        result.maxPositionError = 0.0;
        result.maxEdgeLengthError = 0.0;
        result.maxAreaError = 0.0;
        return;
    }

    std::cout << "    [DEBUG] Mesh pointer is valid, beginning position error computation..." << std::endl;
    std::cout.flush();

    // 计算顶点位置误差
    result.maxPositionError = 0.0;
    size_t vertIdx = 0;

    try {
        for (auto v : mesh1->vertices()) {
            auto pos1 = geom1->vertexPositions[v];

            // 安全检查：确保索引在范围内
            if (vertIdx >= coreMesh->nVertices()) {
                std::cerr << "WARNING: Vertex index out of range: " << vertIdx << " >= " << coreMesh->nVertices() << std::endl;
                break;
            }

            // 获取core网格对应顶点
            VertexPtr v2 = coreMesh->vertex(vertIdx);
            Vector3 pos2 = coreG->position(v2);

            double error = std::sqrt(
                (pos1.x - pos2.x) * (pos1.x - pos2.x) +
                (pos1.y - pos2.y) * (pos1.y - pos2.y) +
                (pos1.z - pos2.z) * (pos1.z - pos2.z)
            );

            result.maxPositionError = std::max(result.maxPositionError, error);
            vertIdx++;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR in position error computation: " << e.what() << std::endl;
        std::cerr << "  Failed at vertex index: " << vertIdx << std::endl;
    }

    // 计算边长误差
    result.maxEdgeLengthError = 0.0;
    size_t edgeIdx = 0;

    try {
        for (auto e : mesh1->edges()) {
            double len1 = geom1->edgeLengths[e];

            // 安全检查：确保索引在范围内
            if (edgeIdx >= coreMesh->nEdges()) {
                std::cerr << "WARNING: Edge index out of range: " << edgeIdx << " >= " << coreMesh->nEdges() << std::endl;
                break;
            }

            EdgePtr e2 = coreMesh->edge(edgeIdx);
            double len2 = coreG->length(e2);

            double error = std::abs(len1 - len2);
            result.maxEdgeLengthError = std::max(result.maxEdgeLengthError, error);
            edgeIdx++;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR in edge length error computation: " << e.what() << std::endl;
        std::cerr << "  Failed at edge index: " << edgeIdx << std::endl;
    }

    // 计算面积误差
    result.maxAreaError = 0.0;
    size_t faceIdx = 0;

    try {
        for (auto f : mesh1->faces()) {
            double area1 = geom1->faceAreas[f];

            // 安全检查：确保索引在范围内
            if (faceIdx >= coreMesh->nFaces()) {
                std::cerr << "WARNING: Face index out of range: " << faceIdx << " >= " << coreMesh->nFaces() << std::endl;
                break;
            }

            FacePtr f2 = coreMesh->face(faceIdx);
            double area2 = coreG->area(f2);

            double error = std::abs(area1 - area2);
            result.maxAreaError = std::max(result.maxAreaError, error);
            faceIdx++;
        }
    } catch (const std::exception& e) {
        std::cerr << "ERROR in area error computation: " << e.what() << std::endl;
        std::cerr << "  Failed at face index: " << faceIdx << std::endl;
    }
}

} // namespace SurfaceTextureMapping
