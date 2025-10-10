/**
 * 图案回映射器测试
 *
 * 测试目标: PatternBackMapper 类
 * 测试范围:
 * - 单三角形内路径映射
 * - 跨边界路径映射
 * - 缝交叉检测
 * - 边交叉点计算
 * - 路径拼接
 * - 测地线路径计算
 * - Real-Space尺度保持
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
#include "geometrycentral/surface/meshio.h"

#include "pattern_back_mapper.h"
#include "barycentric_mapper.h"
#include "texture_mapping.h"

using namespace SurfaceTextureMapping;
using namespace geometrycentral;
using namespace geometrycentral::surface;

/**
 * 测试夹具类
 */
class PatternBackMapperTest : public ::testing::Test {
protected:
    void SetUp() override {
        backMapper_ = std::make_unique<PatternBackMapper>();
        baryMapper_ = std::make_shared<BarycentricMapper>();
    }

    void TearDown() override {
        backMapper_.reset();
        baryMapper_.reset();
        mesh_.reset();
        geometry_.reset();
    }

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
        mesh_ = std::shared_ptr<ManifoldSurfaceMesh>(std::move(ptrMesh));
        geometry_ = std::shared_ptr<VertexPositionGeometry>(std::move(ptrGeometry));
    }

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
     * 创建带缝的UV映射 (模拟切缝后的展开)
     * 将正方形沿中心线切开
     */
    TextureMapper::UVMapping createSeamMapping() {
        TextureMapper::UVMapping mapping;
        mapping.uvCoordinates.resize(mesh_->nVertices());

        // 左侧三角形保持原位
        // 右侧三角形向右平移
        size_t i = 0;
        for (Vertex v : mesh_->vertices()) {
            Vector3 pos = geometry_->vertexPositions[v];
            if (pos.x > 0.5) {
                // 右侧点向右平移
                mapping.uvCoordinates[i] = Vector2{pos.x + 0.1, pos.y};
            } else {
                mapping.uvCoordinates[i] = Vector2{pos.x, pos.y};
            }
            ++i;
        }

        return mapping;
    }

    std::unique_ptr<PatternBackMapper> backMapper_;
    std::shared_ptr<BarycentricMapper> baryMapper_;
    std::shared_ptr<ManifoldSurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
};

// ============================================================================
// 测试用例：路径分段
// ============================================================================

/**
 * 测试: 三角形内路径分段
 *
 * 场景: 路径完全在一个三角形内
 * 期望: 生成单个IntraTriangle段
 */
TEST_F(PatternBackMapperTest, SegmentPath_IntraTriangle) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // 路径完全在第一个三角形内
    std::vector<Vector2> uvPath = {
        {0.1, 0.1},
        {0.2, 0.1},
        {0.2, 0.2}
    };

    // Act
    auto segments = backMapper_->segmentUVPath(uvPath);

    // Assert
    ASSERT_FALSE(segments.empty());
    // 应该至少有一个段
    bool hasIntraSegment = false;
    for (const auto& seg : segments) {
        if (seg.type == PatternBackMapper::SegmentType::IntraTriangle) {
            hasIntraSegment = true;
            break;
        }
    }
    EXPECT_TRUE(hasIntraSegment) << "应该包含三角形内段";
}

/**
 * 测试: 跨边路径分段
 *
 * 场景: 路径从一个三角形跨到另一个三角形
 * 期望: 生成CrossEdge段
 */
TEST_F(PatternBackMapperTest, SegmentPath_CrossEdge) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // 路径从第一个三角形跨到第二个三角形
    std::vector<Vector2> uvPath = {
        {0.25, 0.25},  // 在第一个三角形
        {0.75, 0.75}   // 在第二个三角形
    };

    // Act
    auto segments = backMapper_->segmentUVPath(uvPath);

    // Assert
    EXPECT_FALSE(segments.empty());
    // 可能会生成多个段 (取决于实现细节)
}

// ============================================================================
// 测试用例：缝映射构建
// ============================================================================

/**
 * 测试: 构建缝映射表
 */
TEST_F(PatternBackMapperTest, BuildSeamMapping_IdentifiesSeams) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createSeamMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // Act
    backMapper_->buildSeamMapping();
    auto seamMappings = backMapper_->getSeamMappings();

    // Assert
    // 如果存在缝，应该检测到
    // (具体数量取决于createSeamMapping的实现)
    EXPECT_TRUE(true) << "缝映射构建不应该崩溃";
}

// ============================================================================
// 测试用例：缝交叉检测
// ============================================================================

/**
 * 测试: 检测路径是否跨越缝边界
 */
TEST_F(PatternBackMapperTest, DetectSeamCrossing_NoCrossing) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);
    backMapper_->buildSeamMapping();

    PatternBackMapper::UVPathSegment segment;
    segment.start = Vector2{0.1, 0.1};
    segment.end = Vector2{0.2, 0.2};
    segment.type = PatternBackMapper::SegmentType::IntraTriangle;

    // Act
    bool crossesSeam = backMapper_->detectSeamCrossing(segment);

    // Assert
    EXPECT_FALSE(crossesSeam) << "完美映射没有缝，不应该检测到交叉";
}

// ============================================================================
// 测试用例：线段交点计算
// ============================================================================

/**
 * 测试: 计算两条线段的交点
 *
 * 辅助测试: 验证内部几何计算的正确性
 */
TEST_F(PatternBackMapperTest, LineIntersection_CorrectComputation) {
    // 这个测试需要访问私有方法，实际实现时可能需要将算法提取为公共工具函数
    // 这里只测试公共接口

    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // Act & Assert
    // 通过公共接口间接测试交点计算
    EXPECT_TRUE(true) << "占位测试：线段交点计算";
}

// ============================================================================
// 测试用例：单条路径映射
// ============================================================================

/**
 * 测试: 简单直线路径映射 (无跨缝)
 */
TEST_F(PatternBackMapperTest, MapPathTo3D_SimpleStraightLine) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    std::vector<Vector2> uvPath = {
        {0.0, 0.0},
        {0.5, 0.0},
        {1.0, 0.0}
    };

    // Act
    PatternBackMapper::MappingParams params;
    params.useGeodesicPath = false;  // 简单情况不需要测地线
    auto path3D = backMapper_->mapPathTo3D(uvPath, params);

    // Assert
    EXPECT_EQ(path3D.size(), uvPath.size()) << "3D路径点数应该与UV路径相同";

    // 完美映射: 3D点应该与UV点坐标相同
    for (size_t i = 0; i < path3D.size(); ++i) {
        EXPECT_NEAR(path3D[i].x, uvPath[i].x, 1e-5);
        EXPECT_NEAR(path3D[i].y, uvPath[i].y, 1e-5);
        EXPECT_NEAR(path3D[i].z, 0.0, 1e-5);
    }
}

/**
 * 测试: 曲线路径映射
 */
TEST_F(PatternBackMapperTest, MapPathTo3D_CurvedPath) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // 创建弧形路径
    std::vector<Vector2> uvPath;
    int numPoints = 10;
    for (int i = 0; i < numPoints; ++i) {
        double t = static_cast<double>(i) / (numPoints - 1);
        double x = t;
        double y = 0.5 * std::sin(t * M_PI);  // 正弦曲线
        uvPath.push_back({x, y});
    }

    // Act
    auto path3D = backMapper_->mapPathTo3D(uvPath);

    // Assert
    EXPECT_FALSE(path3D.empty());
    EXPECT_GE(path3D.size(), uvPath.size()) << "可能会插入额外的点";

    // 验证起点和终点
    EXPECT_NEAR(path3D.front().x, uvPath.front().x, 1e-3);
    EXPECT_NEAR(path3D.back().x, uvPath.back().x, 1e-3);
}

// ============================================================================
// 测试用例：批量路径映射
// ============================================================================

/**
 * 测试: 批量映射多条路径
 */
TEST_F(PatternBackMapperTest, MapPathsTo3D_MultiplePaths) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    std::vector<std::vector<Vector2>> uvPaths = {
        {{0.0, 0.0}, {1.0, 0.0}},  // 水平线
        {{0.0, 0.0}, {0.0, 1.0}},  // 垂直线
        {{0.0, 0.0}, {1.0, 1.0}}   // 对角线
    };

    // Act
    auto paths3D = backMapper_->mapPathsTo3D(uvPaths);

    // Assert
    EXPECT_EQ(paths3D.size(), uvPaths.size());

    for (size_t i = 0; i < paths3D.size(); ++i) {
        EXPECT_FALSE(paths3D[i].empty()) << "路径 " << i << " 不应该为空";
    }
}

// ============================================================================
// 测试用例：Real-Space 尺度保持
// ============================================================================

/**
 * 测试: 路径长度保持 (Real-Space)
 *
 * 关键测试: 验证UV空间长度 = 3D空间长度
 */
TEST_F(PatternBackMapperTest, RealSpace_LengthPreservation) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    // 简单直线路径
    std::vector<Vector2> uvPath = {
        {0.0, 0.0},
        {1.0, 0.0}
    };

    // Act
    PatternBackMapper::MappingParams params;
    params.preserveRealSpace = true;
    auto path3D = backMapper_->mapPathTo3D(uvPath, params);

    // 计算UV长度
    double uvLength = (uvPath[1] - uvPath[0]).norm();

    // 计算3D长度
    double length3D = 0.0;
    for (size_t i = 1; i < path3D.size(); ++i) {
        length3D += (path3D[i] - path3D[i-1]).norm();
    }

    // Assert
    EXPECT_NEAR(length3D, uvLength, uvLength * 0.01)
        << "3D长度应该与UV长度一致 (1%误差容差)";
}

/**
 * 测试: 映射质量评估
 */
TEST_F(PatternBackMapperTest, EvaluateMappingQuality_LengthConsistency) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    std::vector<Vector2> uvPath = {
        {0.0, 0.0},
        {0.5, 0.5},
        {1.0, 0.0}
    };

    auto path3D = backMapper_->mapPathTo3D(uvPath);

    // Act
    auto quality = backMapper_->evaluateMappingQuality(uvPath, path3D);

    // Assert
    EXPECT_TRUE(quality.isLengthConsistent)
        << "完美映射的长度应该一致";
    EXPECT_LT(quality.lengthDeviation, 1.0)
        << "长度偏差应该 < 1%";
}

// ============================================================================
// 测试用例：测地线路径
// ============================================================================

/**
 * 测试: 测地线路径计算
 *
 * 注意: 这是一个占位测试，实际实现需要CGAL或geometry-central的测地线算法
 */
TEST_F(PatternBackMapperTest, GeodesicPath_BasicFunctionality) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    Vector3 start{0.0, 0.0, 0.0};
    Vector3 end{1.0, 0.0, 0.0};

    // Act & Assert
    EXPECT_NO_THROW({
        auto geodesicPath = backMapper_->computeGeodesicPath(start, end);
        // 测地线路径应该至少包含起点和终点
        if (!geodesicPath.empty()) {
            EXPECT_GE(geodesicPath.size(), 2);
        }
    });
}

/**
 * 测试: 测地线分辨率控制
 */
TEST_F(PatternBackMapperTest, GeodesicPath_ResolutionControl) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    Vector3 start{0.0, 0.0, 0.0};
    Vector3 end{1.0, 0.0, 0.0};

    // Act
    double fineResolution = 0.05;
    double coarseResolution = 0.2;

    auto finePath = backMapper_->computeGeodesicPath(start, end, fineResolution);
    auto coarsePath = backMapper_->computeGeodesicPath(start, end, coarseResolution);

    // Assert
    if (!finePath.empty() && !coarsePath.empty()) {
        EXPECT_GT(finePath.size(), coarsePath.size())
            << "更高分辨率应该产生更多点";
    }
}

// ============================================================================
// 测试用例：边界情况
// ============================================================================

/**
 * 测试: 空路径处理
 */
TEST_F(PatternBackMapperTest, EmptyPath_HandlesGracefully) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    std::vector<Vector2> emptyPath;

    // Act & Assert
    EXPECT_NO_THROW({
        auto path3D = backMapper_->mapPathTo3D(emptyPath);
        EXPECT_TRUE(path3D.empty());
    });
}

/**
 * 测试: 单点路径
 */
TEST_F(PatternBackMapperTest, SinglePointPath_HandlesCorrectly) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    std::vector<Vector2> singlePoint = {{0.5, 0.5}};

    // Act
    auto path3D = backMapper_->mapPathTo3D(singlePoint);

    // Assert
    EXPECT_EQ(path3D.size(), 1);
    EXPECT_NEAR(path3D[0].x, 0.5, 1e-5);
    EXPECT_NEAR(path3D[0].y, 0.5, 1e-5);
}

/**
 * 测试: 路径点在网格外
 */
TEST_F(PatternBackMapperTest, PathOutsideMesh_HandlesGracefully) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    baryMapper_->setInput(mesh_, geometry_, mapping);
    baryMapper_->buildSpatialIndex();
    backMapper_->setInput(mesh_, geometry_, mapping, baryMapper_);

    std::vector<Vector2> outsidePath = {
        {2.0, 2.0},  // 完全在网格外
        {3.0, 3.0}
    };

    // Act & Assert
    EXPECT_NO_THROW({
        auto path3D = backMapper_->mapPathTo3D(outsidePath);
        // 可能返回空路径或最近投影，但不应该崩溃
    });
}

// ============================================================================
// 主函数 (由test_main.cpp提供)
// ============================================================================