# Surface Texture Mapping - 测试系统文档

版本: 1.0
日期: 2025-09-30

## 概述

本测试系统基于Google Test框架，采用TDD (Test-Driven Development) 方法，为Real-Space UV展开项目提供全面的测试覆盖。

## 测试层次结构

### 第一层：单元测试 (Unit Tests)

测试单个类或函数的功能，不依赖外部模块。

#### 核心新增模块 (高优先级)

1. **test_distortion_analyzer.cpp** - UV失真分析器
   - 雅可比矩阵计算
   - SVD奇异值分解
   - 拉伸失真、共形误差、面积失真
   - 全局统计和颜色映射
   - 失真阈值判定

2. **test_barycentric_mapper.cpp** - 重心坐标映射器
   - 重心坐标计算 (2D/3D)
   - 点在三角形内判定
   - UV<->3D双向映射
   - 批量映射
   - 边界情况处理

3. **test_pattern_back_mapper.cpp** - 图案回映射器
   - 路径分段
   - 缝交叉检测
   - 测地线路径计算
   - Real-Space尺度保持
   - 批量路径映射

#### 基础模块 (待实现)

- test_mesh_processor.cpp
- test_variational_cutter.cpp
- test_texture_mapper.cpp
- test_surface_filler.cpp

### 第二层：集成测试 (Integration Tests)

测试多个模块协同工作的正确性。

1. **test_real_space.cpp** - Real-Space核心特性
   - 单位闭环测试 (mm全链路)
   - 统一比例尺验证
   - 距离和面积保持
   - 图案密度Real-Space解释
   - 多单位转换
   - 端到端尺度一致性

2. **test_uv_pipeline.cpp** (待实现)
   - 完整UV展开流程
   - 迭代优化流程
   - 失真驱动返修

3. **test_cross_seam.cpp** (待实现)
   - 跨缝图案映射
   - 缝映射一致性
   - 测地线路径

### 第三层：端到端测试 (End-to-End Tests)

测试完整的用户工作流程，使用真实数据。

1. **test_e2e_cylinder.cpp** - 圆柱展开
   - 几何验证 (周长、高度)
   - Real-Space尺度验证
   - 失真度量验证
   - 制造容差检查
   - 性能基准

2. **test_e2e_spot.cpp** (待实现)
   - Spot模型完整流程
   - 复杂几何失真验证

3. **test_quality_control.cpp** (待实现)
   - 制造容差检查
   - 质量报告生成
   - PASS/FAIL判定

## 编译和运行

### 前置要求

- CMake 3.10+
- C++17编译器 (MSVC 2022, GCC 9+, Clang 10+)
- Google Test (通过vcpkg安装)
- 项目依赖: geometry-central, Eigen3, Clipper2

### 编译测试

```bash
# 配置构建
cmake --preset stm-debug

# 编译所有测试
cmake --build --preset stm-debug --target all

# 或者只编译测试
cmake --build build/STM-Debug -t test_distortion_analyzer
```

### 运行测试

#### 使用CMake/CTest

```bash
# 运行所有测试
cd build/STM-Debug
ctest --output-on-failure

# 运行特定测试
ctest -R test_distortion_analyzer --verbose

# 运行带标签的测试
ctest -L unit          # 只运行单元测试
ctest -L integration   # 只运行集成测试
ctest -L e2e           # 只运行端到端测试
```

#### 使用Make目标

```bash
# 快速测试 (只运行单元测试)
make test_quick

# 完整测试
make test_all

# 核心模块测试
make test_core

# 集成测试
make test_integration

# 端到端测试
make test_e2e
```

#### 直接运行可执行文件

```bash
cd build/STM-Debug/bin/tests

# 运行所有测试
./test_distortion_analyzer

# 运行特定测试用例
./test_distortion_analyzer --gtest_filter=DistortionAnalyzerTest.PerfectMapping_ZeroDistortion

# 详细输出
./test_distortion_analyzer --gtest_verbose

# 生成XML报告
./test_distortion_analyzer --gtest_output=xml:test_results.xml

# 列出所有测试用例
./test_distortion_analyzer --gtest_list_tests
```

## 测试用例命名规范

遵循Google Test的命名约定：

```
TEST_F(TestFixture, MethodName_Scenario_ExpectedBehavior)
```

示例：
- `PerfectMapping_ZeroDistortion` - 完美映射应该产生零失真
- `StretchedMapping_CorrectSigmaValues` - 拉伸映射应该产生正确的奇异值
- `EmptyMesh_HandlesGracefully` - 空网格应该优雅处理

## 断言规范

### 数值比较

```cpp
EXPECT_NEAR(actual, expected, tolerance);
EXPECT_DOUBLE_EQ(actual, expected);
EXPECT_FLOAT_EQ(actual, expected);
```

### 布尔断言

```cpp
EXPECT_TRUE(condition);
EXPECT_FALSE(condition);
```

### 比较断言

```cpp
EXPECT_EQ(actual, expected);
EXPECT_NE(actual, expected);
EXPECT_LT(value, threshold);
EXPECT_LE(value, threshold);
EXPECT_GT(value, threshold);
EXPECT_GE(value, threshold);
```

### 异常断言

```cpp
EXPECT_THROW(statement, exception_type);
EXPECT_NO_THROW(statement);
EXPECT_ANY_THROW(statement);
```

## 测试数据

### 标准测试网格

测试系统使用以下标准测试网格 (位于 `data/` 目录):

- `simple_triangle.obj` - 单个三角形
- `quad.obj` - 四边形 (2个三角形)
- `cylinder.obj` - 圆柱 (R=10mm, H=50mm)
- `cone.obj` - 圆锥
- `sphere.obj` - 球体 (R=20mm)
- `spot_simplified.obj` - 简化的Spot模型 (~1000面)

### Real-Space参数

所有测试使用Real-Space单位系统：
- **基本单位**: 1 UV单位 = 1 mm (3D空间)
- **失真阈值**:
  - 拉伸失真 < 2.0 (100%拉伸)
  - 共形误差 < 1.5
  - 面积失真 < 2.0
- **制造容差**: < 1mm

## 关键测试场景

### Real-Space单位测试

```cpp
TEST_F(RealSpaceTest, CylinderUnwrap_PreservesCircumference) {
    // 圆柱: R=10mm, H=50mm
    // 期望: UV宽度 = 2πR ≈ 62.83mm
    // 期望: UV高度 = 50mm
    // 容差: < 1%
}
```

### 失真计算测试

```cpp
TEST_F(DistortionTest, PerfectMapping_ZeroDistortion) {
    // 平面网格展开到平面UV
    // 期望: σ_max = σ_min = 1.0
    // 期望: QC = 1.0
    // 期望: area_ratio = 1.0
}
```

### 重心坐标测试

```cpp
TEST_F(BarycentricTest, PointAtCenter_CorrectCoordinates) {
    // 点在三角形中心
    // 期望: 重心坐标 ≈ (1/3, 1/3, 1/3)
}
```

### 迭代优化测试

```cpp
TEST_F(IterationTest, HighDistortion_ConvergesAfterRefinement) {
    // 初始失真 > 阈值
    // 期望: 迭代后失真 < 阈值
    // 期望: 收敛标志 = true
}
```

## 性能基准

### 目标性能

- **小网格** (< 1k面): < 100ms
- **中等网格** (1k-10k面): < 1s
- **大网格** (10k-100k面): < 10s
- **超大网格** (> 100k面): < 60s

### 性能测试

```cpp
TEST_F(PerformanceTest, LargeMesh_ProcessingTime) {
    // 100k面片模型
    // 期望: 总时间 < 60s
    // 期望: 内存 < 2GB
}
```

## 持续集成 (CI)

### 测试报告生成

```bash
# 生成JUnit XML报告 (用于Jenkins/GitLab CI)
make test_report

# 生成代码覆盖率报告 (需要gcov/lcov)
cmake -DENABLE_COVERAGE=ON ..
make coverage
```

### CI配置示例 (GitLab CI)

```yaml
test:
  stage: test
  script:
    - cmake --preset stm-debug
    - cmake --build --preset stm-debug
    - ctest --output-junit test_results.xml
  artifacts:
    reports:
      junit: test_results.xml
```

## 测试覆盖目标

- **单元测试**: > 80% 代码覆盖率
- **集成测试**: 覆盖所有主要工作流程
- **端到端测试**: 覆盖所有用户场景

## 常见问题

### Q: 测试编译失败

A: 检查Google Test是否正确安装:
```bash
find_package(GTest REQUIRED)
```

确保vcpkg已安装gtest:
```bash
vcpkg install gtest
```

### Q: 测试运行失败

A: 检查工作目录和测试数据路径:
```bash
# 测试数据应该在正确的位置
ls data/spot.obj
```

### Q: 如何调试单个测试

A: 使用IDE调试器或gdb:
```bash
gdb --args ./test_distortion_analyzer --gtest_filter=DistortionAnalyzerTest.PerfectMapping_ZeroDistortion
```

## 贡献指南

### 添加新测试

1. 创建测试文件 `test_new_module.cpp`
2. 包含测试夹具类
3. 编写测试用例 (遵循命名规范)
4. 在 `CMakeLists.txt` 中添加测试:
   ```cmake
   add_stm_test(test_new_module test_new_module.cpp)
   ```
5. 运行测试验证

### 测试代码风格

- 使用清晰的测试用例名称
- 遵循AAA模式 (Arrange-Act-Assert)
- 添加注释说明测试目的
- 测试边界情况
- 使用有意义的断言消息

## 参考资料

- [Google Test文档](https://google.github.io/googletest/)
- [CMake CTest文档](https://cmake.org/cmake/help/latest/manual/ctest.1.html)
- 项目技术架构: `TECHNICAL_ARCHITECTURE.md`
- API参考: `API_REFERENCE.md`

## 联系方式

如有问题，请参考项目文档或提交Issue。