/**
 * 测试真实算法实现
 * 验证 VariationalCuttingAlgorithm 和 UVUnwrappingAlgorithm 的功能
 */

#include <iostream>
#include <memory>

// geometry-central 基础库
#include <geometrycentral/surface/halfedge_mesh.h>
#include <geometrycentral/surface/vertex_position_geometry.h>
#include <geometrycentral/surface/meshio.h>

// 真实算法实现
#include "gui/include/real_algorithms.h"

using namespace geometrycentral;
using namespace geometrycentral::surface;
using namespace SurfaceTextureMapping;

int main(int argc, char* argv[]) {
    std::cout << "=== 真实算法测试程序 ===" << std::endl;

    // 默认测试网格文件路径
    std::string meshFile = "data/spot.obj";
    if (argc > 1) {
        meshFile = argv[1];
    }

    std::cout << "加载网格文件: " << meshFile << std::endl;

    try {
        // 加载网格
        std::unique_ptr<HalfedgeMesh> mesh;
        std::unique_ptr<VertexPositionGeometry> geometry;
        std::tie(mesh, geometry) = readManifoldSurfaceMesh(meshFile);

        if (!mesh || !geometry) {
            std::cerr << "错误：无法加载网格文件 " << meshFile << std::endl;
            return 1;
        }

        std::cout << "成功加载网格："
                  << mesh->nVertices() << " 顶点, "
                  << mesh->nFaces() << " 面" << std::endl;

        // 测试算法流水线
        AlgorithmPipeline pipeline;

        if (!pipeline.setMesh(mesh.get(), geometry.get())) {
            std::cerr << "错误：无法设置网格到算法流水线" << std::endl;
            return 1;
        }

        std::cout << "\n=== 开始算法测试 ===" << std::endl;

        // 步骤1：网格分析
        std::cout << "\n--- 步骤1：网格分析 ---" << std::endl;
        MeshAnalysis analysis = pipeline.runMeshAnalysis();
        std::cout << "分析完成" << std::endl;

        // 步骤2：特征检测
        std::cout << "\n--- 步骤2：特征检测 ---" << std::endl;
        auto features = pipeline.runFeatureDetection(0.8);
        std::cout << "检测到 " << features.size() << " 个特征点" << std::endl;

        // 步骤3：初始切割生成
        std::cout << "\n--- 步骤3：初始切割生成 ---" << std::endl;
        auto cuts = pipeline.runInitialCutGeneration();
        std::cout << "生成了 " << cuts.size() << " 条切割路径" << std::endl;

        // 步骤4：切割优化
        if (!cuts.empty()) {
            std::cout << "\n--- 步骤4：切割优化 ---" << std::endl;
            pipeline.runCutOptimization(5); // 减少迭代次数以加快测试
            std::cout << "切割优化完成" << std::endl;
        }

        // 步骤5：UV展开
        std::cout << "\n--- 步骤5：UV展开 ---" << std::endl;
        bool uvSuccess = pipeline.runUVUnwrapping();
        if (uvSuccess) {
            std::cout << "UV展开成功" << std::endl;
        } else {
            std::cout << "UV展开失败" << std::endl;
        }

        // 获取可视化数据
        auto visData = pipeline.getVisualizationData();

        std::cout << "\n=== 算法结果摘要 ===" << std::endl;
        std::cout << "网格顶点数: " << visData.vertices.size() << std::endl;
        std::cout << "网格面数: " << visData.faces.size() << std::endl;
        std::cout << "特征点数: " << visData.featurePoints.size() << std::endl;
        std::cout << "切割线数: " << visData.cutLines.size() << std::endl;
        std::cout << "UV坐标数: " << visData.uvCoordinates.size() << std::endl;
        std::cout << "UV失真度: " << visData.uvDistortion << std::endl;

        // 网格质量统计
        std::cout << "\n=== 网格质量统计 ===" << std::endl;
        std::cout << "欧拉特征数: " << visData.analysis.eulerCharacteristic << std::endl;
        std::cout << "亏格: " << visData.analysis.genus << std::endl;
        std::cout << "表面积: " << visData.analysis.surfaceArea << std::endl;
        std::cout << "平均边长: " << visData.analysis.avgEdgeLength << std::endl;
        std::cout << "平均高斯曲率: " << visData.analysis.avgGaussianCurvature << std::endl;
        std::cout << "平均三角形质量: " << visData.analysis.avgTriangleQuality << std::endl;
        std::cout << "是否流形: " << (visData.analysis.isManifold ? "是" : "否") << std::endl;
        std::cout << "是否封闭: " << (visData.analysis.isWatertight ? "是" : "否") << std::endl;

        std::cout << "\n=== 测试完成 ===" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "异常: " << e.what() << std::endl;
        return 1;
    }
}