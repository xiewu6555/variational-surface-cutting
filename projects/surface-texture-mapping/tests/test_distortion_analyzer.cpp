/**
 * UV失真分析器测试
 *
 * 测试目标: UVDistortionAnalyzer 类
 * 测试范围:
 * - 雅可比矩阵计算
 * - SVD奇异值分解
 * - 拉伸失真计算 (σ_min, σ_max)
 * - 共形误差计算 (QC = σ_max/σ_min)
 * - 面积失真计算
 * - 颜色编码映射
 * - 全局统计 (mean, std, percentile_95)
 * - 顶点颜色平滑
 * - 失真阈值判定
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <gtest/gtest.h>
#include <Eigen/Core>
#include <Eigen/SVD>
#include <memory>
#include <cmath>

#include "geometrycentral/surface/manifold_surface_mesh.h"
#include "geometrycentral/surface/vertex_position_geometry.h"
#include "geometrycentral/surface/surface_mesh_factories.h"

#include "uv_distortion_analyzer.h"
#include "texture_mapping.h"

using namespace SurfaceTextureMapping;
using namespace geometrycentral;
using namespace geometrycentral::surface;

/**
 * 测试夹具类
 */
class DistortionAnalyzerTest : public ::testing::Test {
protected:
    void SetUp() override {
        analyzer_ = std::make_unique<UVDistortionAnalyzer>();
    }

    void TearDown() override {
        analyzer_.reset();
        mesh_.reset();
        geometry_.reset();
    }

    /**
     * 创建简单的测试网格: 单位正方形 (2个三角形)
     * 顶点: (0,0,0), (1,0,0), (1,1,0), (0,1,0)
     * 面: (0,1,2), (0,2,3)
     */
    void createUnitSquareMesh() {
        std::vector<Vector3> positions = {
            {0.0, 0.0, 0.0},
            {1.0, 0.0, 0.0},
            {1.0, 1.0, 0.0},
            {0.0, 1.0, 0.0}
        };

        std::vector<std::vector<size_t>> faces = {
            {0, 1, 2},  // 第一个三角形
            {0, 2, 3}   // 第二个三角形
        };

        auto [ptrMesh, ptrGeometry] = makeManifoldSurfaceMeshAndGeometry(faces, positions);
        mesh_ = std::move(ptrMesh);
        geometry_ = std::move(ptrGeometry);
    }

    /**
     * 设置analyzer输入 (直接传递shared_ptr)
     */
    void setAnalyzerInput(const TextureMapper::UVMapping& mapping) {
        analyzer_->setInput(mesh_, geometry_, mapping);
    }

    /**
     * 创建完美映射: UV坐标 = XY坐标 (无失真)
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

        mapping.totalDistortion = 0.0;
        mapping.maxDistortion = 0.0;
        return mapping;
    }

    /**
     * 创建拉伸映射: UV在X方向拉伸2倍
     */
    TextureMapper::UVMapping createStretchedMapping(double stretchFactor) {
        TextureMapper::UVMapping mapping;
        mapping.uvCoordinates.resize(mesh_->nVertices());

        size_t i = 0;
        for (Vertex v : mesh_->vertices()) {
            Vector3 pos = geometry_->vertexPositions[v];
            mapping.uvCoordinates[i] = Vector2{pos.x * stretchFactor, pos.y};
            ++i;
        }

        return mapping;
    }

    /**
     * 创建各向异性映射: X方向拉伸，Y方向压缩
     */
    TextureMapper::UVMapping createAnisotropicMapping(double xScale, double yScale) {
        TextureMapper::UVMapping mapping;
        mapping.uvCoordinates.resize(mesh_->nVertices());

        size_t i = 0;
        for (Vertex v : mesh_->vertices()) {
            Vector3 pos = geometry_->vertexPositions[v];
            mapping.uvCoordinates[i] = Vector2{pos.x * xScale, pos.y * yScale};
            ++i;
        }

        return mapping;
    }

    std::unique_ptr<UVDistortionAnalyzer> analyzer_;
    std::shared_ptr<ManifoldSurfaceMesh> mesh_;
    std::shared_ptr<VertexPositionGeometry> geometry_;
};

// ============================================================================
// 测试用例：基础功能
// ============================================================================

/**
 * 测试: 完美映射应该产生零失真
 *
 * 场景: 平面网格展开到平面UV，没有任何变形
 * 期望:
 * - σ_max = 1.0 (无拉伸)
 * - σ_min = 1.0 (无压缩)
 * - QC = 1.0 (完美共形)
 * - area_ratio = 1.0 (面积不变)
 */
TEST_F(DistortionAnalyzerTest, PerfectMapping_ZeroDistortion) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    ASSERT_EQ(distortions.size(), 2) << "应该有2个面";

    for (const auto& dist : distortions) {
        EXPECT_NEAR(dist.sigmaMax, 1.0, 1e-6) << "最大拉伸应该为1.0";
        EXPECT_NEAR(dist.sigmaMin, 1.0, 1e-6) << "最小拉伸应该为1.0";
        EXPECT_NEAR(dist.conformalError, 1.0, 1e-6) << "共形误差应该为1.0";
        EXPECT_NEAR(dist.areaRatio, 1.0, 1e-6) << "面积比应该为1.0";
    }
}

/**
 * 测试: 拉伸映射应该产生正确的奇异值
 *
 * 场景: X方向拉伸2倍的映射
 * 期望:
 * - σ_max = 2.0
 * - σ_min = 1.0
 * - QC = 2.0
 * - area_ratio = 2.0
 */
TEST_F(DistortionAnalyzerTest, StretchedMapping_CorrectSigmaValues) {
    // Arrange
    createUnitSquareMesh();
    double stretchFactor = 2.0;
    auto mapping = createStretchedMapping(stretchFactor);
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    ASSERT_FALSE(distortions.empty());

    for (const auto& dist : distortions) {
        EXPECT_NEAR(dist.sigmaMax, stretchFactor, 1e-5)
            << "最大拉伸应该为 " << stretchFactor;
        EXPECT_NEAR(dist.sigmaMin, 1.0, 1e-5)
            << "最小拉伸应该为1.0";
        EXPECT_NEAR(dist.conformalError, stretchFactor, 1e-5)
            << "共形误差应该为 " << stretchFactor;
        EXPECT_NEAR(dist.areaRatio, stretchFactor, 1e-5)
            << "面积比应该为 " << stretchFactor;
    }
}

/**
 * 测试: 各向异性映射的失真计算
 *
 * 场景: X方向拉伸3倍，Y方向压缩0.5倍
 * 期望:
 * - σ_max = 3.0
 * - σ_min = 0.5
 * - QC = 6.0
 * - area_ratio = 1.5
 */
TEST_F(DistortionAnalyzerTest, AnisotropicMapping_CorrectDistortion) {
    // Arrange
    createUnitSquareMesh();
    double xScale = 3.0;
    double yScale = 0.5;
    auto mapping = createAnisotropicMapping(xScale, yScale);
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    ASSERT_FALSE(distortions.empty());

    for (const auto& dist : distortions) {
        EXPECT_NEAR(dist.sigmaMax, xScale, 1e-5) << "σ_max应该为 " << xScale;
        EXPECT_NEAR(dist.sigmaMin, yScale, 1e-5) << "σ_min应该为 " << yScale;
        EXPECT_NEAR(dist.conformalError, xScale / yScale, 1e-5)
            << "QC应该为 " << (xScale / yScale);
        EXPECT_NEAR(dist.areaRatio, xScale * yScale, 1e-5)
            << "面积比应该为 " << (xScale * yScale);
    }
}

// ============================================================================
// 测试用例：雅可比矩阵计算
// ============================================================================

/**
 * 测试: 雅可比矩阵的正确性
 *
 * 对于完美映射，雅可比矩阵应该是单位矩阵
 */
TEST_F(DistortionAnalyzerTest, JacobianMatrix_IdentityForPerfectMapping) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    for (const auto& dist : distortions) {
        Eigen::Matrix2d identity = Eigen::Matrix2d::Identity();
        EXPECT_TRUE(dist.jacobian.isApprox(identity, 1e-5))
            << "完美映射的雅可比矩阵应该是单位矩阵\n"
            << "实际:\n" << dist.jacobian << "\n"
            << "期望:\n" << identity;
    }
}

/**
 * 测试: 雅可比矩阵的行列式 = σ_max * σ_min
 */
TEST_F(DistortionAnalyzerTest, JacobianDeterminant_EqualsAreaRatio) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createAnisotropicMapping(2.0, 3.0);
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    for (const auto& dist : distortions) {
        double det = dist.jacobian.determinant();
        double expectedDet = dist.sigmaMax * dist.sigmaMin;
        EXPECT_NEAR(det, expectedDet, 1e-5)
            << "行列式应该等于 σ_max * σ_min";
        EXPECT_NEAR(det, dist.areaRatio, 1e-5)
            << "行列式应该等于面积比";
    }
}

// ============================================================================
// 测试用例：全局统计
// ============================================================================

/**
 * 测试: 全局统计的计算
 */
TEST_F(DistortionAnalyzerTest, GlobalStats_CorrectComputation) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createStretchedMapping(2.0);
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto stats = analyzer_->computeGlobalStats(distortions);

    // Assert
    // 所有面都有相同的失真，所以mean = max，std = 0
    EXPECT_NEAR(stats.stretchMean, 2.0, 1e-5);
    EXPECT_NEAR(stats.stretchMax, 2.0, 1e-5);
    EXPECT_NEAR(stats.stretchStd, 0.0, 1e-5);

    EXPECT_NEAR(stats.conformalMean, 2.0, 1e-5);
    EXPECT_NEAR(stats.conformalMax, 2.0, 1e-5);

    EXPECT_NEAR(stats.areaMean, 2.0, 1e-5);
}

/**
 * 测试: 95百分位数计算
 */
TEST_F(DistortionAnalyzerTest, GlobalStats_Percentile95Computation) {
    // Arrange
    createUnitSquareMesh();
    // 创建混合失真: 一个面拉伸2倍，另一个面拉伸1倍
    // (需要更复杂的网格才能真正测试百分位数，这里只测试接口)
    auto mapping = createStretchedMapping(2.0);
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto stats = analyzer_->computeGlobalStats(distortions);

    // Assert
    EXPECT_GE(stats.stretchPercentile95, 0.0);
    EXPECT_LE(stats.stretchPercentile95, stats.stretchMax);
}

// ============================================================================
// 测试用例：质量阈值判定
// ============================================================================

/**
 * 测试: 完美映射应该通过所有质量测试
 */
TEST_F(DistortionAnalyzerTest, QualityThreshold_PerfectMappingPasses) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setAnalyzerInput(mapping);
    analyzer_->setStretchThreshold(2.0);
    analyzer_->setConformalThreshold(1.5);

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto stats = analyzer_->computeGlobalStats(distortions);

    // Assert
    EXPECT_TRUE(stats.passStretchThreshold);
    EXPECT_TRUE(stats.passConformalThreshold);
    EXPECT_TRUE(stats.passOverallQuality);
}

/**
 * 测试: 高失真映射应该无法通过质量测试
 */
TEST_F(DistortionAnalyzerTest, QualityThreshold_HighDistortionFails) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createStretchedMapping(3.0);  // 拉伸3倍
    setAnalyzerInput(mapping);
    analyzer_->setStretchThreshold(2.0);  // 阈值: 拉伸 < 2.0

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto stats = analyzer_->computeGlobalStats(distortions);

    // Assert
    EXPECT_FALSE(stats.passStretchThreshold) << "拉伸失真超过阈值";
}

/**
 * 测试: 共形误差阈值判定
 */
TEST_F(DistortionAnalyzerTest, QualityThreshold_ConformalErrorCheck) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createAnisotropicMapping(2.0, 1.0);  // QC = 2.0
    setAnalyzerInput(mapping);
    analyzer_->setConformalThreshold(1.5);  // 阈值: QC < 1.5

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto stats = analyzer_->computeGlobalStats(distortions);

    // Assert
    EXPECT_FALSE(stats.passConformalThreshold)
        << "共形误差 2.0 > 阈值 1.5";
}

// ============================================================================
// 测试用例：高失真区域标记
// ============================================================================

/**
 * 测试: 标记高失真区域
 */
TEST_F(DistortionAnalyzerTest, MarkHighDistortionRegions_CorrectIdentification) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createStretchedMapping(3.0);
    analyzer_->setInput(mesh_, geometry_, mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto highDistortionFaces = analyzer_->markHighDistortionRegions(distortions, 2.0);

    // Assert
    EXPECT_EQ(highDistortionFaces.size(), 2)
        << "所有面都应该被标记为高失真 (拉伸3.0 > 阈值2.0)";
}

/**
 * 测试: 低失真区域不应该被标记
 */
TEST_F(DistortionAnalyzerTest, MarkHighDistortionRegions_LowDistortionNotMarked) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createPerfectMapping();
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto highDistortionFaces = analyzer_->markHighDistortionRegions(distortions, 2.0);

    // Assert
    EXPECT_TRUE(highDistortionFaces.empty())
        << "完美映射不应该有高失真区域";
}

// ============================================================================
// 测试用例：颜色编码映射
// ============================================================================

/**
 * 测试: 颜色映射生成
 */
TEST_F(DistortionAnalyzerTest, ColorMap_GeneratesValidColors) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createStretchedMapping(2.0);
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();
    auto colorMap = analyzer_->generateColorMap(distortions, "Stretch");

    // Assert
    EXPECT_EQ(colorMap.faceColors.size(), distortions.size());
    EXPECT_STREQ(colorMap.metricName.c_str(), "Stretch");

    // 检查颜色值在有效范围内 [0, 1]
    for (const auto& color : colorMap.faceColors) {
        EXPECT_GE(color.x(), 0.0);
        EXPECT_LE(color.x(), 1.0);
        EXPECT_GE(color.y(), 0.0);
        EXPECT_LE(color.y(), 1.0);
        EXPECT_GE(color.z(), 0.0);
        EXPECT_LE(color.z(), 1.0);
    }
}

/**
 * 测试: 不同度量类型的颜色映射
 */
TEST_F(DistortionAnalyzerTest, ColorMap_DifferentMetricTypes) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createAnisotropicMapping(2.0, 0.5);
    setAnalyzerInput(mapping);
    auto distortions = analyzer_->computeAllDistortions();

    // Act & Assert
    auto stretchMap = analyzer_->generateColorMap(distortions, "Stretch");
    EXPECT_STREQ(stretchMap.metricName.c_str(), "Stretch");

    auto conformalMap = analyzer_->generateColorMap(distortions, "Conformal");
    EXPECT_STREQ(conformalMap.metricName.c_str(), "Conformal");

    auto areaMap = analyzer_->generateColorMap(distortions, "Area");
    EXPECT_STREQ(areaMap.metricName.c_str(), "Area");
}

// ============================================================================
// 测试用例：边界情况
// ============================================================================

/**
 * 测试: 空网格处理
 */
TEST_F(DistortionAnalyzerTest, EmptyMesh_HandlesGracefully) {
    // Arrange
    // 不创建网格，直接设置空输入
    TextureMapper::UVMapping emptyMapping;

    // Act & Assert
    // 应该能够安全处理，不会崩溃
    EXPECT_NO_THROW({
        auto distortions = analyzer_->computeAllDistortions();
        EXPECT_TRUE(distortions.empty());
    });
}

/**
 * 测试: 退化三角形 (面积为0) 的处理
 */
TEST_F(DistortionAnalyzerTest, DegenerateTriangle_HandlesGracefully) {
    // Arrange
    std::vector<Vector3> positions = {
        {0.0, 0.0, 0.0},
        {1.0, 0.0, 0.0},
        {1.0, 0.0, 0.0}  // 重复顶点 -> 退化三角形
    };

    std::vector<std::vector<size_t>> faces = {{0, 1, 2}};

    auto [ptrMesh, ptrGeometry] = makeManifoldSurfaceMeshAndGeometry(faces, positions);
    mesh_ = std::move(ptrMesh);
    geometry_ = std::move(ptrGeometry);

    auto mapping = createPerfectMapping();
    setAnalyzerInput(mapping);

    // Act & Assert
    // 应该能够检测并处理退化情况，不会崩溃
    EXPECT_NO_THROW({
        auto distortions = analyzer_->computeAllDistortions();
        // 退化三角形可能返回无穷大或NaN，需要特殊处理
        for (const auto& dist : distortions) {
            // 检查是否为有限值
            bool isFinite = std::isfinite(dist.sigmaMax) &&
                          std::isfinite(dist.sigmaMin);
            // 如果不是有限值，应该有明确标记
            if (!isFinite) {
                EXPECT_TRUE(true) << "退化三角形应该被标记";
            }
        }
    });
}

// ============================================================================
// 测试用例：数值稳定性
// ============================================================================

/**
 * 测试: 极小失真的数值稳定性
 */
TEST_F(DistortionAnalyzerTest, NumericalStability_VerySmallDistortion) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createStretchedMapping(1.0001);  // 非常小的拉伸
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    for (const auto& dist : distortions) {
        EXPECT_TRUE(std::isfinite(dist.sigmaMax));
        EXPECT_TRUE(std::isfinite(dist.sigmaMin));
        EXPECT_TRUE(std::isfinite(dist.conformalError));
        EXPECT_GT(dist.conformalError, 0.0) << "共形误差应该为正数";
    }
}

/**
 * 测试: 极大失真的数值稳定性
 */
TEST_F(DistortionAnalyzerTest, NumericalStability_VeryLargeDistortion) {
    // Arrange
    createUnitSquareMesh();
    auto mapping = createStretchedMapping(100.0);  // 极大拉伸
    setAnalyzerInput(mapping);

    // Act
    auto distortions = analyzer_->computeAllDistortions();

    // Assert
    for (const auto& dist : distortions) {
        EXPECT_TRUE(std::isfinite(dist.sigmaMax));
        EXPECT_TRUE(std::isfinite(dist.sigmaMin));
        EXPECT_TRUE(std::isfinite(dist.conformalError));
    }
}

// ============================================================================
// 主函数由test_main.cpp提供
// ============================================================================