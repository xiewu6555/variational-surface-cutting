/**
 * 重心坐标映射器测试
 *
 * 测试目标: BarycentricMapper 类
 * 测试范围:
 * - 点在三角形内判定
 * - 重心坐标计算
 * - 3D点插值
 * - UV三角形定位
 * - 边界情况 (点在边上、顶点上)
 * - 数值稳定性 (epsilon处理)
 * - UV<->3D双向映射
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <memory>
#include <cmath>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"

#include "barycentric_mapper.h"
#include "texture_mapping.h"

using namespace SurfaceTextureMapping;
using namespace geometrycentral;
using namespace geometrycentral::surface;

/**
 * 测试夹具类
 */
class BarycentricMapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        mapper_ = std::make_unique<BarycentricMapper>();
    }

    void TearDown() override {
        mapper_.reset();
        mesh_.reset();
        geometry_.reset();
    }

    /**
     * 创建标准单位三角形
     * 3D: (0,0,0), (1,0,0), (0,1,0)
     * UV: 与3D相同 (完美映射)
     */
    void createUnitTriangleMesh() {
        std::vector<Vector3> positions = {
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {0.0, 1.0, 0.0}
        };

        std::vector<std::vector<size_t>> faces = {{0, 1, 2}};

        auto [ptrMesh, ptrGeometry] = makeManifoldSurfaceMeshAndGeometry(faces, positions);
        mesh_ = std::move(ptrMesh);
        geometry_ = std::move(ptrGeometry);
    }

    /**
     * 创建单位正方形网格 (2个三角形)
     */
    void createUnitSquareMesh() {
        std::vector<Vector3> positions = {
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {0.0, 1.0, 0.0}
        };

        std::vector<std::vector<size_t>> faces = {
            {0, 1, 2},
            {0, 2, 3}
        };

        auto [ptrMesh, ptrGeometry] = makeManifoldSurfaceMeshAndGeometry(faces, positions);
        mesh_ = std::move(ptrMesh);
        geometry_ = std::move(ptrGeometry);
    }

    /**
     * 设置mapper输入 (直接传递shared_ptr)
     */
    void setMapperInput(const TextureMapper::UVMapping& mapping) {
        mapper_->setInput(mesh_, geometry_, mapping);
    }

    /**
     * 创建完美UV映射 (UV = XY)
     */
    TextureMapper::UVMapping createPerfectMapping() {
        TextureMapper::UVMapping mapping;
        mapping.uvCoordinates.resize(mesh_->nVertices());

        size_t i = 0;
        for (Vertex v : mesh_->vertices()) {
            Vector3 pos = geometry_->vertexPositions[v];
            mapping.uvCoordinates[i] = Vector2{pos.x, pos.y};
            ++i;
        }

        return mapping;
    }

    /**
     * 创建拉伸UV映射 (UV = 2*XY)
     */
    TextureMapper::UVMapping createStretchedMapping(double scale) {
        TextureMapper::UVMapping mapping;
        mapping.uvCoordinates.resize(mesh_->nVertices());

        size_t i = 0;
        for (Vertex v : mesh_->vertices()) {
            Vector3 pos = geometry_->vertexPositions[v];
            mapping.uvCoordinates[i] = Vector2{pos.x * scale, pos.y * scale};
            ++i;
        }

        return mapping;
    }

    std::unique_ptr<BarycentricMapper> mapper_;
    std::shared_ptr<ManifoldSurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
};

// ============================================================================
// 测试用例：重心坐标计算 (2D)
// ============================================================================

/**
 * 测试: 点在顶点上的重心坐标
 *
 * 场景: 点正好在三角形的一个顶点上
 * 期望: 重心坐标为 (1, 0, 0) 或 (0, 1, 0) 或 (0, 0, 1)
 */
TEST_F(BarycentricMapperTest, BarycentricCoord2D_PointAtVertex) {
    // Arrange
    Eigen::Vector2d v0(0.0, 0.0);
    Eigen::Vector2d v1(1.0, 0.0);
    Eigen::Vector2d v2(0.0, 1.0);

    // Act & Assert - 点在第一个顶点
    Eigen::Vector3d bary0 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, v0);
    EXPECT_NEAR(bary0.x(), 1.0, 1e-10);
    EXPECT_NEAR(bary0.y(), 0.0, 1e-10);
    EXPECT_NEAR(bary0.z(), 0.0, 1e-10);

    // 点在第二个顶点
    Eigen::Vector3d bary1 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, v1);
    EXPECT_NEAR(bary1.x(), 0.0, 1e-10);
    EXPECT_NEAR(bary1.y(), 1.0, 1e-10);
    EXPECT_NEAR(bary1.z(), 0.0, 1e-10);

    // 点在第三个顶点
    Eigen::Vector3d bary2 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, v2);
    EXPECT_NEAR(bary2.x(), 0.0, 1e-10);
    EXPECT_NEAR(bary2.y(), 0.0, 1e-10);
    EXPECT_NEAR(bary2.z(), 1.0, 1e-10);
}

/**
 * 测试: 点在三角形中心的重心坐标
 *
 * 场景: 点在三角形的重心位置
 * 期望: 重心坐标约为 (1/3, 1/3, 1/3)
 */
TEST_F(BarycentricMapperTest, BarycentricCoord2D_PointAtCenter) {
    // Arrange
    Eigen::Vector2d v0(0.0, 0.0);
    Eigen::Vector2d v1(1.0, 0.0);
    Eigen::Vector2d v2(0.0, 1.0);
    Eigen::Vector2d center = (v0 + v1 + v2) / 3.0;

    // Act
    Eigen::Vector3d bary = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, center);

    // Assert
    EXPECT_NEAR(bary.x(), 1.0 / 3.0, 1e-10);
    EXPECT_NEAR(bary.y(), 1.0 / 3.0, 1e-10);
    EXPECT_NEAR(bary.z(), 1.0 / 3.0, 1e-10);
    EXPECT_NEAR(bary.sum(), 1.0, 1e-10);
}

/**
 * 测试: 点在边上的重心坐标
 *
 * 场景: 点在三角形某条边的中点
 * 期望: 相应的重心坐标为 (0.5, 0.5, 0) 等
 */
TEST_F(BarycentricMapperTest, BarycentricCoord2D_PointOnEdge) {
    // Arrange
    Eigen::Vector2d v0(0.0, 0.0);
    Eigen::Vector2d v1(1.0, 0.0);
    Eigen::Vector2d v2(0.0, 1.0);

    // Act & Assert - 点在边 v0-v1 的中点
    Eigen::Vector2d midpoint01 = (v0 + v1) / 2.0;
    Eigen::Vector3d bary01 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, midpoint01);
    EXPECT_NEAR(bary01.x(), 0.5, 1e-10);
    EXPECT_NEAR(bary01.y(), 0.5, 1e-10);
    EXPECT_NEAR(bary01.z(), 0.0, 1e-10);

    // 点在边 v0-v2 的中点
    Eigen::Vector2d midpoint02 = (v0 + v2) / 2.0;
    Eigen::Vector3d bary02 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, midpoint02);
    EXPECT_NEAR(bary02.x(), 0.5, 1e-10);
    EXPECT_NEAR(bary02.y(), 0.0, 1e-10);
    EXPECT_NEAR(bary02.z(), 0.5, 1e-10);

    // 点在边 v1-v2 的中点
    Eigen::Vector2d midpoint12 = (v1 + v2) / 2.0;
    Eigen::Vector3d bary12 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, midpoint12);
    EXPECT_NEAR(bary12.x(), 0.0, 1e-10);
    EXPECT_NEAR(bary12.y(), 0.5, 1e-10);
    EXPECT_NEAR(bary12.z(), 0.5, 1e-10);
}

/**
 * 测试: 重心坐标的和总是为1
 */
TEST_F(BarycentricMapperTest, BarycentricCoord2D_SumIsOne) {
    // Arrange
    Eigen::Vector2d v0(0.0, 0.0);
    Eigen::Vector2d v1(1.0, 0.0);
    Eigen::Vector2d v2(0.0, 1.0);

    // Act - 测试多个随机点
    std::vector<Eigen::Vector2d> testPoints = {
        {0.1, 0.1}, {0.2, 0.3}, {0.5, 0.2}, {0.3, 0.3}, {0.8, 0.05}
    };

    // Assert
    for (const auto& point : testPoints) {
        Eigen::Vector3d bary = BarycentricMapper::computeBarycentricCoordinates(
            v0, v1, v2, point);
        EXPECT_NEAR(bary.sum(), 1.0, 1e-10)
            << "点 (" << point.x() << ", " << point.y() << ") 的重心坐标和不为1";
    }
}

// ============================================================================
// 测试用例：重心坐标计算 (3D)
// ============================================================================

/**
 * 测试: 3D空间的重心坐标计算
 */
TEST_F(BarycentricMapperTest, BarycentricCoord3D_PointAtVertex) {
    // Arrange
    Eigen::Vector3d v0(0.0, 0.0, 0.0);
    Eigen::Vector3d v1(1.0, 0.0, 0.0);
    Eigen::Vector3d v2(0.0, 1.0, 0.0);

    // Act
    Eigen::Vector3d bary0 = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, v0);

    // Assert
    EXPECT_NEAR(bary0.x(), 1.0, 1e-10);
    EXPECT_NEAR(bary0.y(), 0.0, 1e-10);
    EXPECT_NEAR(bary0.z(), 0.0, 1e-10);
}

/**
 * 测试: 3D空间点在三角形中心
 */
TEST_F(BarycentricMapperTest, BarycentricCoord3D_PointAtCenter) {
    // Arrange
    Eigen::Vector3d v0(0.0, 0.0, 0.0);
    Eigen::Vector3d v1(1.0, 0.0, 0.0);
    Eigen::Vector3d v2(0.0, 1.0, 0.0);
    Eigen::Vector3d center = (v0 + v1 + v2) / 3.0;

    // Act
    Eigen::Vector3d bary = BarycentricMapper::computeBarycentricCoordinates(
        v0, v1, v2, center);

    // Assert
    EXPECT_NEAR(bary.x(), 1.0 / 3.0, 1e-10);
    EXPECT_NEAR(bary.y(), 1.0 / 3.0, 1e-10);
    EXPECT_NEAR(bary.z(), 1.0 / 3.0, 1e-10);
}

// ============================================================================
// 测试用例：点在三角形内判定
// ============================================================================

/**
 * 测试: 点在三角形内 (所有重心坐标为正)
 */
TEST_F(BarycentricMapperTest, IsPointInTriangle_InteriorPoint) {
    // Arrange
    Eigen::Vector3d interiorBary(0.3, 0.4, 0.3);

    // Act
    bool isInside = BarycentricMapper::isPointInTriangle(interiorBary);

    // Assert
    EXPECT_TRUE(isInside) << "内部点应该在三角形内";
}

/**
 * 测试: 点在三角形边界上
 */
TEST_F(BarycentricMapperTest, IsPointInTriangle_OnBoundary) {
    // Arrange
    Eigen::Vector3d boundaryBary(0.5, 0.5, 0.0);  // 在边上

    // Act
    bool isInside = BarycentricMapper::isPointInTriangle(boundaryBary);

    // Assert
    EXPECT_TRUE(isInside) << "边界点应该被认为在三角形内";
}

/**
 * 测试: 点在三角形外 (某个重心坐标为负)
 */
TEST_F(BarycentricMapperTest, IsPointInTriangle_OutsidePoint) {
    // Arrange
    Eigen::Vector3d outsideBary(0.5, 0.7, -0.2);  // 负坐标 -> 在外部

    // Act
    bool isInside = BarycentricMapper::isPointInTriangle(outsideBary);

    // Assert
    EXPECT_FALSE(isInside) << "外部点不应该在三角形内";
}

/**
 * 测试: 数值容差处理 (epsilon边界)
 */
TEST_F(BarycentricMapperTest, IsPointInTriangle_EpsilonTolerance) {
    // Arrange
    double eps = 1e-11;
    Eigen::Vector3d almostInsideBary(0.0 - eps, 0.5, 0.5 + eps);  // 稍微超出边界

    // Act
    bool isInside = BarycentricMapper::isPointInTriangle(almostInsideBary, 1e-10);

    // Assert
    EXPECT_TRUE(isInside)
        << "在epsilon容差内的点应该被认为在三角形内";
}

// ============================================================================
// 测试用例：UV -> 3D 映射
// ============================================================================

/**
 * 测试: UV点到3D点的映射 (完美映射)
 */
TEST_F(BarycentricMapperTest, MapUVto3D_PerfectMapping) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - UV中心点应该映射到3D中心点
    Vector2 uvCenter{0.5, 0.5};
    auto result = mapper_->mapUVto3D(uvCenter);

    // Assert
    ASSERT_TRUE(result.has_value()) << "映射应该成功";
    EXPECT_TRUE(result->success);
    EXPECT_NEAR(result->point3D.x, 0.5, 1e-5);
    EXPECT_NEAR(result->point3D.y, 0.5, 1e-5);
    EXPECT_NEAR(result->point3D.z, 0.0, 1e-5);
}

/**
 * 测试: UV点到3D点的映射 (拉伸映射)
 */
TEST_F(BarycentricMapperTest, MapUVto3D_StretchedMapping) {
    // Arrange
    createUnitSquareMesh();
    double scale = 2.0;
    auto mapping = createStretchedMapping(scale);
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - UV空间的点应该映射回原始3D位置
    Vector2 uvPoint{1.0, 1.0};  // UV中的(1,1)对应3D中的(0.5, 0.5)
    auto result = mapper_->mapUVto3D(uvPoint);

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->success);
    // 由于UV是拉伸的，UV(1,1)应该对应3D(0.5, 0.5)
    EXPECT_NEAR(result->point3D.x, 0.5, 1e-5);
    EXPECT_NEAR(result->point3D.y, 0.5, 1e-5);
}

/**
 * 测试: UV点在三角形顶点上
 */
TEST_F(BarycentricMapperTest, MapUVto3D_PointAtVertex) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - UV顶点应该精确映射到3D顶点
    Vector2 uvVertex{1.0, 0.0};
    auto result = mapper_->mapUVto3D(uvVertex);

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->success);
    EXPECT_NEAR(result->point3D.x, 1.0, 1e-10);
    EXPECT_NEAR(result->point3D.y, 0.0, 1e-10);
    EXPECT_NEAR(result->point3D.z, 0.0, 1e-10);
}

/**
 * 测试: UV点在三角形边上
 */
TEST_F(BarycentricMapperTest, MapUVto3D_PointOnEdge) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - UV边中点应该映射到3D边中点
    Vector2 uvEdgeMidpoint{0.5, 0.0};
    auto result = mapper_->mapUVto3D(uvEdgeMidpoint);

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->success);
    EXPECT_NEAR(result->point3D.x, 0.5, 1e-10);
    EXPECT_NEAR(result->point3D.y, 0.0, 1e-10);
}

// ============================================================================
// 测试用例：3D -> UV 映射
// ============================================================================

/**
 * 测试: 3D点到UV点的映射
 */
TEST_F(BarycentricMapperTest, Map3DtoUV_PerfectMapping) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - 3D中心点应该映射到UV中心点
    Vector3 point3D{0.5, 0.5, 0.0};
    auto result = mapper_->map3DtoUV(point3D);

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->success);
    EXPECT_NEAR(result->pointUV.x, 0.5, 1e-5);
    EXPECT_NEAR(result->pointUV.y, 0.5, 1e-5);
}

/**
 * 测试: 3D点在顶点上
 */
TEST_F(BarycentricMapperTest, Map3DtoUV_PointAtVertex) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act
    Vector3 vertex3D{1.0, 0.0, 0.0};
    auto result = mapper_->map3DtoUV(vertex3D);

    // Assert
    ASSERT_TRUE(result.has_value());
    EXPECT_TRUE(result->success);
    EXPECT_NEAR(result->pointUV.x, 1.0, 1e-10);
    EXPECT_NEAR(result->pointUV.y, 0.0, 1e-10);
}

// ============================================================================
// 测试用例：批量映射
// ============================================================================

/**
 * 测试: 批量UV到3D映射
 */
TEST_F(BarycentricMapperTest, BatchMapUVto3D_MultiplePoints) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    std::vector<Vector2> uvPoints = {
        {0.25, 0.25},
        {0.5, 0.5},
        {0.75, 0.25}
    };

    // Act
    auto results = mapper_->batchMapUVto3D(uvPoints);

    // Assert
    ASSERT_EQ(results.size(), uvPoints.size());

    for (size_t i = 0; i < results.size(); ++i) {
        EXPECT_TRUE(results[i].success)
            << "点 " << i << " 映射失败";
        // 完美映射: 3D = UV
        EXPECT_NEAR(results[i].point3D.x, uvPoints[i].x, 1e-5);
        EXPECT_NEAR(results[i].point3D.y, uvPoints[i].y, 1e-5);
    }
}

/**
 * 测试: 批量3D到UV映射
 */
TEST_F(BarycentricMapperTest, BatchMap3DtoUV_MultiplePoints) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    std::vector<Vector3> points3D = {
        {0.25, 0.25, 0.0},
        {0.5, 0.5, 0.0},
        {0.75, 0.25, 0.0}
    };

    // Act
    auto results = mapper_->batchMap3DtoUV(points3D);

    // Assert
    ASSERT_EQ(results.size(), points3D.size());

    for (size_t i = 0; i < results.size(); ++i) {
        EXPECT_TRUE(results[i].success);
        EXPECT_NEAR(results[i].pointUV.x, points3D[i].x, 1e-5);
        EXPECT_NEAR(results[i].pointUV.y, points3D[i].y, 1e-5);
    }
}

// ============================================================================
// 测试用例：插值函数
// ============================================================================

/**
 * 测试: 3D点插值
 */
TEST_F(BarycentricMapperTest, Interpolate3DPoint_CorrectResult) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);

    // 获取第一个面
    Face face = *mesh_->faces().begin();

    // Act - 使用重心(1/3, 1/3, 1/3)插值
    Eigen::Vector3d bary(1.0/3.0, 1.0/3.0, 1.0/3.0);
    Vector3 interpolated = mapper_->interpolate3DPoint(face, bary);

    // Assert - 应该得到三角形的中心点
    Vector3 expectedCenter{1.0/3.0, 1.0/3.0, 0.0};
    EXPECT_NEAR(interpolated.x, expectedCenter.x, 1e-10);
    EXPECT_NEAR(interpolated.y, expectedCenter.y, 1e-10);
    EXPECT_NEAR(interpolated.z, expectedCenter.z, 1e-10);
}

/**
 * 测试: UV点插值
 */
TEST_F(BarycentricMapperTest, InterpolateUVPoint_CorrectResult) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);

    Face face = *mesh_->faces().begin();

    // Act
    Eigen::Vector3d bary(0.5, 0.5, 0.0);  // 边中点
    Vector2 interpolated = mapper_->interpolateUVPoint(face, bary);

    // Assert
    EXPECT_NEAR(interpolated.x, 0.5, 1e-10);
    EXPECT_NEAR(interpolated.y, 0.0, 1e-10);
}

// ============================================================================
// 测试用例：三角形查找
// ============================================================================

/**
 * 测试: 查找包含UV点的三角形
 */
TEST_F(BarycentricMapperTest, FindUVTriangle_PointInside) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - 查找包含中心点的三角形
    Vector2 centerUV{0.5, 0.5};
    auto face = mapper_->findUVTriangle(centerUV);

    // Assert
    EXPECT_TRUE(face.has_value()) << "应该找到包含该点的三角形";
}

/**
 * 测试: UV点在网格外
 */
TEST_F(BarycentricMapperTest, FindUVTriangle_PointOutside) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);
    mapper_->buildSpatialIndex();

    // Act - 查找网格外的点
    Vector2 outsideUV{2.0, 2.0};
    auto face = mapper_->findUVTriangle(outsideUV);

    // Assert
    EXPECT_FALSE(face.has_value()) << "网格外的点不应该找到三角形";
}

// ============================================================================
// 测试用例：边界情况处理
// ============================================================================

/**
 * 测试: 边捕捉功能
 */
TEST_F(BarycentricMapperTest, BoundaryHandling_SnapToEdge) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);

    BarycentricMapper::BoundaryHandling handling;
    handling.snapToEdge = true;
    handling.snapTolerance = 1e-3;
    mapper_->setBoundaryHandling(handling);

    // Act - 点非常接近边
    Vector2 nearEdge{0.5001, 0.0001};  // 接近底边
    auto result = mapper_->mapUVto3D(nearEdge);

    // Assert
    EXPECT_TRUE(result.has_value());
    // 应该捕捉到边上
}

/**
 * 测试: 顶点捕捉功能
 */
TEST_F(BarycentricMapperTest, BoundaryHandling_SnapToVertex) {
    // Arrange
    createUnitTriangleMesh();
    auto mapping = createPerfectMapping();
    setMapperInput(mapping);

    BarycentricMapper::BoundaryHandling handling;
    handling.snapToVertex = true;
    handling.snapTolerance = 1e-3;
    mapper_->setBoundaryHandling(handling);

    // Act - 点非常接近顶点
    Vector2 nearVertex{0.0001, 0.0001};
    auto result = mapper_->mapUVto3D(nearVertex);

    // Assert
    EXPECT_TRUE(result.has_value());
    // 应该捕捉到顶点
}

// ============================================================================
// 测试用例：数值稳定性
// ============================================================================

/**
 * 测试: 极小三角形的处理
 */
TEST_F(BarycentricMapperTest, NumericalStability_TinyTriangle) {
    // Arrange
    std::vector<Vector3> positions = {
        {0.0, 0.0, 0.0},
        {1e-8, 0.0, 0.0},
        {0.0, 1e-8, 0.0}
    };

    std::vector<std::vector<size_t>> faces = {{0, 1, 2}};
    auto [ptrMesh, ptrGeometry] = makeManifoldSurfaceMeshAndGeometry(faces, positions);
    mesh_ = std::move(ptrMesh);
    geometry_ = std::move(ptrGeometry);

    auto mapping = createPerfectMapping();
    setMapperInput(mapping);

    // Act & Assert - should not crash
    mapper_->buildSpatialIndex();
    Vector2 uvPoint{5e-9, 5e-9};
    auto result = mapper_->mapUVto3D(uvPoint);
    // May fail, but should not crash
    EXPECT_TRUE(true);  // Test passes if we reach here without crashing
}

// ============================================================================
// 主函数由test_main.cpp提供
// ============================================================================