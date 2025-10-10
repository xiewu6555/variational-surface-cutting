#pragma once

#include <string>
#include <vector>
#include <optional>
#include <memory>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"

namespace SurfaceTextureMapping {

/**
 * 网格预处理类
 * 负责网格的基础修复、重网格化和流形化
 */
class MeshProcessor {
public:
    using SurfaceMesh = geometrycentral::surface::ManifoldSurfaceMesh;
    using VertexPositionGeometry = geometrycentral::surface::VertexPositionGeometry;

    MeshProcessor() = default;
    ~MeshProcessor() = default;

    /**
     * 加载网格文件
     * @param filename 网格文件路径（支持OBJ、PLY等格式）
     * @return 成功返回true，失败返回false
     */
    bool loadMesh(const std::string& filename);

    /**
     * 保存网格文件
     * @param filename 输出文件路径
     * @return 成功返回true，失败返回false
     */
    bool saveMesh(const std::string& filename) const;

    /**
     * 基础网格修复
     * - 焊接重复顶点
     * - 统一法向
     * - 移除孤立片段
     * @return 修复是否成功
     */
    bool basicRepair();

    /**
     * 等各向性重网格化
     * @param targetEdgeLength 目标边长
     * @param iterations 迭代次数
     * @param protectBoundary 是否保护边界
     * @return 重网格化是否成功
     */
    bool isotropicRemeshing(double targetEdgeLength,
                           int iterations = 3,
                           bool protectBoundary = true);

    /**
     * 流形化检查和修复
     * @return 是否成功转换为流形网格
     */
    bool makeManifold();

    /**
     * 获取网格统计信息
     */
    struct MeshStats {
        size_t numVertices;
        size_t numFaces;
        size_t numEdges;
        double minEdgeLength;
        double maxEdgeLength;
        double avgEdgeLength;
        bool isManifold;
        bool isClosed;
    };

    MeshStats getMeshStats() const;

    // Getter方法
    std::shared_ptr<SurfaceMesh> getMesh() const { return mesh_; }
    std::shared_ptr<VertexPositionGeometry> getGeometry() const { return geometry_; }

private:
    std::shared_ptr<SurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;

    /**
     * 计算合适的目标边长
     * @return 推荐的目标边长
     */
    double computeTargetEdgeLength() const;

    // 私有辅助方法
    void removeDuplicateVertices();
    void removeDegenerateFaces();
    void fixNormalOrientation();
    void splitLongEdges(double threshold);
    void collapseShortEdges(double threshold);
    void flipEdges();
    void smoothVertices();
    void projectToOriginalSurface();
    void removeNonManifoldVertices();
    void removeNonManifoldEdges();
    void fillSmallHoles(int maxHoleSize);
};

} // namespace SurfaceTextureMapping