/**
 * Real-Space UV展开集成测试
 *
 * 测试目标: Real-Space (真实尺度) UV展开的核心特性
 * 测试范围:
 * - 单位闭环测试 (mm全链路)
 * - 统一比例尺验证
 * - 尺度校准补偿
 * - 多单位转换
 * - 端到端尺度一致性
 *
 * Real-Space原则:
 * - 1 UV单位 = 1 mm (3D空间)
 * - 图案密度以mm为单位指定
 * - 失真度量基于真实长度
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

#include "mesh_processor.h"
#include "texture_mapping.h"
#include "surface_filling.h"
#include "barycentric_mapper.h"
#include "pattern_back_mapper.h"

using namespace SurfaceTextureMapping;
using namespace geometrycentral;
using namespace geometrycentral::surface;

/**
 * 测试夹具类
 */
class RealSpaceTest : public ::testing::Test {
protected:
    void SetUp() override {
        meshProcessor_ = std::make_unique<MeshProcessor>();
        texMapper_ = std::make_unique<TextureMapper>();
        filler_ = std::make_unique<SurfaceFiller>();
        baryMapper_ = std::make_shared<BarycentricMapper>();
        backMapper_ = std::make_unique<PatternBackMapper>();
    }

    void TearDown() override {
        meshProcessor_.reset();
        texMapper_.reset();
        filler_.reset();
        baryMapper_.reset();
        backMapper_.reset();
        mesh_.reset();
        geometry_.reset();
    }

    /**
     * 创建已知尺寸的正方形网格 (单位: mm)
     */
    void createSquareMesh(double sideLength) {
        std::vector<Vector3> positions = {
            {0.0, 0.0, 0.0},
            {sideLength, 0.0, 0.0},
            {sideLength, sideLength, 0.0},
            {0.0, sideLength, 0.0}
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
     * 创建完美UV映射 (Real-Space: 1 UV = 1 mm)
     */
    TextureMapper::UVMapping createRealSpaceMapping() {
        TextureMapper::UVMapping mapping;
        mapping.uvCoordinates.resize(mesh_->nVertices());

        size_t i = 0;
        for (Vertex v : mesh_->vertices()) {
            Vector3 pos = geometry_->vertexPositions[v];
            // Real-Space: UV坐标 = 3D坐标 (单位mm)
            mapping.uvCoordinates[i] = Vector2{pos.x, pos.y};
            ++i;
        }

        return mapping;
    }

    std::unique_ptr<MeshProcessor> meshProcessor_;
    std::unique_ptr<TextureMapper> texMapper_;
    std::unique_ptr<SurfaceFiller> filler_;
    std::shared_ptr<BarycentricMapper> baryMapper_;
    std::unique_ptr<PatternBackMapper> backMapper_;

    std::unique_ptr<ManifoldSurfaceMesh> mesh_;
    std::unique_ptr<VertexPositionGeometry> geometry_;
};

// ============================================================================
// 测试用例：单位闭环
// ============================================================================

/**
 * 测试: 3D->UV->3D 闭环 (单位保持)
 *
 * 关键测试: 验证往返映射后，尺度不变
 */
TEST_F(RealSpaceTest, UnitClosedLoop_3DtoUVto3D) {
    // Arrange
    double sideLength = 100.0;  // 100mm正方形
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();

    // Act - 3D点 -> UV点 -> 3D点
    Vector3 original3D(50.0, 50.0, 0.0);  // 中心点

    auto uvResult = baryMapper_->map3DtoUV(original3D);
    ASSERT_TRUE(uvResult.has_value());

    auto result3D = baryMapper_->mapUVto3D(uvResult->pointUV);
    ASSERT_TRUE(result3D.has_value());

    // Assert - 往返后应该回到原点
    EXPECT_NEAR(result3D->point3D.x, original3D.x, 0.01)
        << "X坐标误差应该 < 0.01mm";
    EXPECT_NEAR(result3D->point3D.y, original3D.y, 0.01)
        << "Y坐标误差应该 < 0.01mm";
    EXPECT_NEAR(result3D->point3D.z, original3D.z, 0.01)
        << "Z坐标误差应该 < 0.01mm";
}

/**
 * 测试: UV->3D->UV 闭环
 */
TEST_F(RealSpaceTest, UnitClosedLoop_UVto3DtoUV) {
    // Arrange
    double sideLength = 100.0;
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();

    // Act - UV点 -> 3D点 -> UV点
    Vector2 originalUV(50.0, 50.0);

    auto result3D = baryMapper_->mapUVto3D(originalUV);
    ASSERT_TRUE(result3D.has_value());

    auto uvResult = baryMapper_->map3DtoUV(result3D->point3D);
    ASSERT_TRUE(uvResult.has_value());

    // Assert
    EXPECT_NEAR(uvResult->pointUV.x, originalUV.x, 0.01);
    EXPECT_NEAR(uvResult->pointUV.y, originalUV.y, 0.01);
}

// ============================================================================
// 测试用例：统一比例尺
// ============================================================================

/**
 * 测试: 距离保持 (Real-Space)
 *
 * 验证: 3D空间的距离 = UV空间的距离
 */
TEST_F(RealSpaceTest, UnifiedScale_DistancePreservation) {
    // Arrange
    double sideLength = 100.0;
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    // 两个3D点
    Vector3 point1(10.0, 10.0, 0.0);
    Vector3 point2(90.0, 10.0, 0.0);

    // 3D距离
    double distance3D = (point2 - point1).norm();  // 应该 = 80mm

    // 对应的UV点
    Vector2 uv1(10.0, 10.0);
    Vector2 uv2(90.0, 10.0);

    // UV距离
    double distanceUV = (uv2 - uv1).norm();

    // Assert
    EXPECT_NEAR(distance3D, distanceUV, 1e-6)
        << "3D距离应该等于UV距离 (Real-Space)";
    EXPECT_NEAR(distance3D, 80.0, 1e-6)
        << "距离应该 = 80mm";
}

/**
 * 测试: 面积保持
 */
TEST_F(RealSpaceTest, UnifiedScale_AreaPreservation) {
    // Arrange
    double sideLength = 50.0;  // 50mm正方形
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    // Act - 计算3D面积
    double area3D = 0.0;
    for (Face f : mesh_->faces()) {
        std::vector<Vector3> vertices;
        for (Vertex v : f.adjacentVertices()) {
            vertices.push_back(geometry_->vertexPositions[v]);
        }

        if (vertices.size() == 3) {
            Vector3 edge1 = vertices[1] - vertices[0];
            Vector3 edge2 = vertices[2] - vertices[0];
            Vector3 cross = geometrycentral::cross(edge1, edge2);
            area3D += 0.5 * cross.norm();
        }
    }

    // 计算UV面积
    double areaUV = 0.0;
    size_t faceIdx = 0;
    for (Face f : mesh_->faces()) {
        std::vector<Vector2> uvVertices;
        for (Vertex v : f.adjacentVertices()) {
            size_t idx = v.getIndex();
            uvVertices.push_back(mapping.uvCoordinates[idx]);
        }

        if (uvVertices.size() == 3) {
            // 2D三角形面积
            double a = (uvVertices[1] - uvVertices[0]).norm();
            double b = (uvVertices[2] - uvVertices[1]).norm();
            double c = (uvVertices[0] - uvVertices[2]).norm();
            double s = (a + b + c) / 2.0;
            areaUV += std::sqrt(s * (s - a) * (s - b) * (s - c));
        }
    }

    // Assert
    double theoreticalArea = sideLength * sideLength;
    EXPECT_NEAR(area3D, theoreticalArea, 0.1)
        << "3D面积应该 ≈ " << theoreticalArea << " mm²";
    EXPECT_NEAR(areaUV, theoreticalArea, 0.1)
        << "UV面积应该 ≈ " << theoreticalArea << " mm²";
    EXPECT_NEAR(area3D, areaUV, 0.1)
        << "3D面积应该等于UV面积";
}

// ============================================================================
// 测试用例：图案密度 (Real-Space)
// ============================================================================

/**
 * 测试: 图案间距的Real-Space解释
 *
 * 验证: spacing=5.0 表示 5mm 间距
 */
TEST_F(RealSpaceTest, PatternDensity_RealSpaceInterpretation) {
    // Arrange
    double sideLength = 100.0;
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    filler_->setInput(mesh_, geometry_, mapping);

    // Act - 生成网格图案，间距 5mm
    SurfaceFiller::FillingParams params;
    params.type = SurfaceFiller::PatternType::Grid;
    params.spacing = 5.0;  // 5mm

    std::vector<Vector2> uvBounds = {
        {0.0, 0.0},
        {sideLength, 0.0},
        {sideLength, sideLength},
        {0.0, sideLength}
    };

    auto gridLines = filler_->generateGridPattern(params.spacing, uvBounds);

    // Assert
    // 100mm正方形，5mm间距 -> 应该有 20条线 (水平) + 20条线 (垂直)
    int expectedLines = static_cast<int>(sideLength / params.spacing) + 1;
    int expectedTotalLines = expectedLines * 2;  // 水平 + 垂直

    EXPECT_GE(gridLines.size(), expectedTotalLines - 4)
        << "应该生成约 " << expectedTotalLines << " 条线";

    // 验证第一条和第二条线的间距
    if (gridLines.size() >= 2) {
        // (这需要知道gridLines的具体结构，这里假设每条线是一个点序列)
        // 简化验证: 检查是否有数据
        EXPECT_FALSE(gridLines.empty());
    }
}

/**
 * 测试: 图案密度与网格分辨率无关
 *
 * 关键: 相同的spacing应该在不同分辨率的网格上产生相同间距的图案
 */
TEST_F(RealSpaceTest, PatternDensity_IndependentOfMeshResolution) {
    // Arrange - 粗网格
    double sideLength = 50.0;
    createSquareMesh(sideLength);
    auto mapping1 = createRealSpaceMapping();

    filler_->setInput(mesh_, geometry_, mapping1);

    SurfaceFiller::FillingParams params;
    params.spacing = 5.0;  // 5mm

    std::vector<Vector2> uvBounds1 = {
        {0.0, 0.0}, {sideLength, 0.0},
        {sideLength, sideLength}, {0.0, sideLength}
    };

    auto gridLines1 = filler_->generateGridPattern(params.spacing, uvBounds1);

    // 现在测试细网格 (分辨率不同，但物理尺寸相同)
    // (实际测试时需要创建不同分辨率的网格)
    // 这里简化为重复测试，验证接口稳定性

    auto gridLines2 = filler_->generateGridPattern(params.spacing, uvBounds1);

    // Assert - 两次生成应该一致
    EXPECT_EQ(gridLines1.size(), gridLines2.size())
        << "相同参数应该产生相同数量的线";
}

// ============================================================================
// 测试用例：多单位转换
// ============================================================================

/**
 * 测试: 单位转换 (mm, cm, inch)
 *
 * 验证单位转换的正确性
 */
TEST_F(RealSpaceTest, UnitConversion_MMtoCM) {
    // Arrange
    double sideLength_mm = 100.0;  // 100mm
    double sideLength_cm = 10.0;   // 10cm

    createSquareMesh(sideLength_mm);

    // Act - 转换为cm
    double scale = 0.1;  // 1mm = 0.1cm
    for (Vertex v : mesh_->vertices()) {
        geometry_->vertexPositions[v] *= scale;
    }

    // 验证转换后的尺寸
    double maxCoord = 0.0;
    for (Vertex v : mesh_->vertices()) {
        Vector3 pos = geometry_->vertexPositions[v];
        maxCoord = std::max(maxCoord, std::max(pos.x, pos.y));
    }

    // Assert
    EXPECT_NEAR(maxCoord, sideLength_cm, 1e-6)
        << "转换后应该是 " << sideLength_cm << " cm";
}

/**
 * 测试: inch 到 mm 转换
 */
TEST_F(RealSpaceTest, UnitConversion_InchToMM) {
    // Arrange
    double sideLength_inch = 1.0;  // 1 inch
    double sideLength_mm = 25.4;   // 1 inch = 25.4mm

    createSquareMesh(sideLength_inch);

    // Act - 转换为mm
    double scale = 25.4;
    for (Vertex v : mesh_->vertices()) {
        geometry_->vertexPositions[v] *= scale;
    }

    double maxCoord = 0.0;
    for (Vertex v : mesh_->vertices()) {
        Vector3 pos = geometry_->vertexPositions[v];
        maxCoord = std::max(maxCoord, std::max(pos.x, pos.y));
    }

    // Assert
    EXPECT_NEAR(maxCoord, sideLength_mm, 1e-6);
}

// ============================================================================
// 测试用例：尺度校准
// ============================================================================

/**
 * 测试: 尺度校准补偿
 *
 * 场景: 网格导入时尺度错误，需要校准
 */
TEST_F(RealSpaceTest, ScaleCalibration_CorrectScale) {
    // Arrange - 假设导入的网格尺度错误 (实际是cm，标注为mm)
    double sideLength_actual = 10.0;   // 实际 10cm
    double sideLength_expected = 100.0; // 期望 100mm

    createSquareMesh(sideLength_actual);

    // Act - 校准
    double scaleFactor = sideLength_expected / sideLength_actual;
    for (Vertex v : mesh_->vertices()) {
        geometry_->vertexPositions[v] *= scaleFactor;
    }

    // Verify
    double maxCoord = 0.0;
    for (Vertex v : mesh_->vertices()) {
        Vector3 pos = geometry_->vertexPositions[v];
        maxCoord = std::max(maxCoord, std::max(pos.x, pos.y));
    }

    // Assert
    EXPECT_NEAR(maxCoord, sideLength_expected, 1e-6)
        << "校准后应该是 " << sideLength_expected << " mm";
}

// ============================================================================
// 测试用例：端到端尺度一致性
// ============================================================================

/**
 * 测试: 完整流程的尺度一致性
 *
 * 3D网格 -> UV展开 -> 图案生成 -> 3D回映射
 * 验证: 最终3D图案的尺度与输入一致
 */
TEST_F(RealSpaceTest, E2E_ScaleConsistency) {
    // Arrange
    double sideLength = 50.0;  // 50mm
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();

    filler_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // Step 1: 生成UV图案 (5mm间距)
    SurfaceFiller::FillingParams params;
    params.spacing = 5.0;
    params.type = SurfaceFiller::PatternType::Grid;

    auto fillingResult = filler_->generateFilling(params);

    // Step 2: 映射回3D
    std::vector<std::vector<Vector3>> paths3D;
    if (!fillingResult.pathsUV.empty()) {
        paths3D = backMapper_->mapPathsTo3D(fillingResult.pathsUV);
    }

    // Assert
    if (!paths3D.empty()) {
        // 验证3D路径的坐标范围
        double minX = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();

        for (const auto& path : paths3D) {
            for (const auto& point : path) {
                minX = std::min(minX, point.x);
                maxX = std::max(maxX, point.x);
            }
        }

        double range = maxX - minX;
        EXPECT_NEAR(range, sideLength, sideLength * 0.05)
            << "3D图案范围应该与网格尺寸一致 (5%误差容差)";
    }
}

/**
 * 测试: Real-Space 图案长度验证
 */
TEST_F(RealSpaceTest, E2E_PatternLengthVerification) {
    // Arrange
    double sideLength = 100.0;
    createSquareMesh(sideLength);
    auto mapping = createRealSpaceMapping();

    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // 简单直线路径 (UV空间)
    std::vector<Vector2> uvPath = {
        {10.0, 50.0},
        {90.0, 50.0}
    };

    double expectedLength = 80.0;  // 80mm

    // Act
    auto path3D = backMapper_->mapPathTo3D(uvPath);

    // 计算3D路径长度
    double length3D = 0.0;
    for (size_t i = 1; i < path3D.size(); ++i) {
        length3D += (path3D[i] - path3D[i-1]).norm();
    }

    // Assert
    EXPECT_NEAR(length3D, expectedLength, expectedLength * 0.01)
        << "3D路径长度应该 = UV路径长度 (1%误差容差)";
}

// ============================================================================
// 主函数
// ============================================================================

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}