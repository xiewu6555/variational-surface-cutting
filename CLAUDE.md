# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## 项目概述

这是一个计算几何研究项目，包含两个主要组件：
1. **Variational Surface Cutting** - 基于变分方法的曲面切割算法 (研究代码)
2. **Surface Texture Mapping** - 完整的曲面纹理映射系统 (现代化实现)

项目采用模块化架构，核心算法基于离散微分几何和优化理论。

## 构建和开发命令

### CMake 构建系统

项目使用现代CMake (3.10+) 和预设配置：

```bash
# 查看可用预设
cmake --list-presets

# 配置 Surface Texture Mapping Debug 构建
cmake --preset stm-debug

# 构建项目
cmake --build --preset stm-debug

# Release 构建
cmake --preset stm-release
cmake --build --preset stm-release
```

### 可执行文件

构建后生成的主要可执行文件（位于 `build/STM-Debug/bin/Debug/`）：
- `cuts.exe` - 原始变分切割GUI应用 (可用 `run_cuts.bat` 启动)
- `SurfaceTextureMapping.exe` - 命令行纹理映射工具
- `SurfaceTextureMapping_gui.exe` - NanoGUI版本的纹理映射GUI
- `SurfaceTextureMapping_imgui_gui.exe` - ImGui版本的纹理映射GUI (推荐)
- `test_spot.exe` - Spot模型测试程序

### 运行和测试

```bash
# 使用批处理文件运行原始Cuts GUI
./run_cuts.bat
# 或直接指定模型
./run_cuts.bat data/spot.obj

# 运行ImGui版本的Surface Texture Mapping GUI (推荐)
cd build/STM-Debug/bin/Debug
./SurfaceTextureMapping_imgui_gui.exe

# 命令行纹理映射工具
./build/STM-Debug/bin/Debug/SurfaceTextureMapping.exe -r 0.01 -c -t -f grid --spacing 0.02 input.obj

# 测试Spot模型
cd build/STM-Debug/bin/Debug
./test_spot.exe
```

### 基本工作流程 - 原始Variational Cuts

基于README.md，典型的变分切割工作流程：

1. 运行GUI: `./run_cuts.bat data/spot.obj`
2. 打开变分切割UI: *Tool Chest --> Variational Cuts*
3. 初始化切割: *Initialize --> Normal Clustering*
4. 设置Hencky能量权重为`3.0`: *Cuts Parameters --> Hencky Distortion --> Weight*
5. 设置步数为`300`: *Cuts Control --> # steps*
6. 运行算法: *Cuts Control --> Take Many Steps*
7. 可视化结果: *Cuts Control --> Show boundary, Show Extra Cuts*
8. 查看参数化: *Cuts Control --> Visualization --> Cut Mesh Param*
9. 导出结果: *Cuts Control --> Save --> Save .obj with injective texcoords*

## 系统架构

模块角色分析

Core 模块 - 底层数学和几何基础库
角色: 提供所有几何算法的数学基础设施
主要组件:
- 半边数据结构 (halfedge_mesh.h/cpp) - 核心网格表示
- 几何计算引擎 (geometry.h/cpp) - 度量、曲率等几何量计算
- 稀疏线性代数 (sparse_matrix.h, solvers.h) - 支持Eigen和SuiteSparse双后端
- 离散微分几何 (discrete_operators.h) - 拉普拉斯算子、梯度等
- 向量和矩阵类 (vector2/3.h, dense_matrix.h) - 基础数学类型
- 网格I/O (meshio.h) - OBJ等格式支持

GUI 模块 - 通用可视化和交互框架
角色: 为所有项目提供统一的3D渲染和用户交互界面
主要组件:
- 3D渲染器 (viewer.h/cpp) - OpenGL渲染管线
- 场景对象 (scene_mesh.h, scene_points.h 等) - 不同几何对象的可视化
- 相机控制 (camera.h/cpp) - 视点操作
- 项目GUI框架 (project_gui.h/cpp) - 可扩展的项目特定界面
- 着色器管理 (shaders/, shaders.h) - GPU着色器程序

Projects 模块 - 具体算法实现和应用
角色: 基于Core和GUI实现具体的几何处理算法
子项目结构:
- cuts - 变分切缝算法 (有独立GUI)
- flatten - 曲面展开算法 (仅核心实现)
- surface-texture-mapping - 现代纹理映射系统 (独立完整项目)

依赖关系分析

● 分层依赖架构
[Projects] - 具体算法实现层
↓ 依赖
[GUI] - 可视化交互层
↓ 依赖
[Core] - 数学几何基础层
↓ 依赖
[外部依赖] - Eigen、SuiteSparse、NanoGUI、OpenGL


### 核心层次结构

```
Core Libraries (core/)
├── Halfedge Mesh         # 半边数据结构 (核心网格表示)
├── Geometry Engine       # 几何计算 (度量、曲率、微分算子)
├── Sparse Linear Algebra # 稀疏矩阵和求解器 (Eigen/SuiteSparse双后端)
└── Discrete DiffGeo      # 离散微分几何算子

Algorithm Layer (projects/*/core/)
├── Variational Cutting   # 变分切割优化 (欧拉形状优化器)
├── Texture Mapping       # UV参数化 (BFF, 共形映射)
├── Mesh Processing       # 网格预处理 (重网格化, 修复)
└── Surface Filling       # 表面图案填充 (空间填充曲线)

Application Layer
├── GUI Framework         # OpenGL渲染和交互
├── CLI Tools            # 命令行工具
└── Testing Utilities    # 测试和基准程序
```

### 关键设计模式

**半边数据结构**: 核心网格表示，支持高效拓扑遍历
- `HalfedgeMesh` 类提供连通性信息
- 智能指针系统 (`VertexPtr`, `EdgePtr`, `FacePtr`) 实现类型安全访问
- 模板化数据关联 (`VertexData<T>`, `FaceData<T>`) 用于存储几何和算法数据

**几何计算引擎**: `Geometry<T>` 模板类
- 支持 `Euclidean` (3D), `Planar` (2D), `Spherical` 几何类型
- 懒惰求值和缓存机制优化性能
- 统一的微分几何量计算接口

**稀疏线性代数**: 双后端设计
- Eigen backend: 纯C++实现，无外部依赖
- SuiteSparse backend: 高性能，适合大规模问题
- 统一接口支持不同求解器 (Cholesky, LU, QR, 特征值)

### Surface Texture Mapping 项目架构

专门的纹理映射系统，位于 `projects/surface-texture-mapping/`:

**核心模块**:
- `MeshProcessor`: 网格预处理和重网格化
- `VariationalCutter`: 基于Yamabe方程的变分切割
- `TextureMapper`: BFF参数化和UV映射
- `SurfaceFiller`: 多种空间填充图案生成

**算法特性**:
- 等各向性重网格化确保数值稳定性
- 变分切割不限制于网格边，支持最优边界位置
- 高质量UV参数化，低失真展平
- 多种填充图案：网格、六边形、螺旋、希尔伯特曲线

## 开发注意事项

### 依赖管理
项目使用git submodule管理依赖：
```bash
# 初始化子模块
git submodule update --init --recursive
```

主要依赖包括：
- geometry-central: 离散微分几何库
- libigl: 网格处理工具
- CGAL: 计算几何算法
- nanogui: GUI框架
- Eigen3: 线性代数 (通过vcpkg)

### 构建配置
- 使用 vcpkg 作为包管理器（通过 `vcpkg.json` 定义依赖）
- CMakeUserPresets.json 配置了专门的构建预设（stm-debug, stm-release, stm-relwithdebinfo）
- 支持 Visual Studio 2022 x64 架构
- C++17 标准
- 主要vcpkg依赖：eigen3, glew, opengl, glfw3, imgui[glfw-binding,opengl3-binding]

### 代码风格
- 现代C++17特性
- 模板化设计实现代码复用
- RAII和智能指针管理资源
- 遵循几何处理领域的数学命名约定

### 性能考虑
- 连续内存布局的数据结构
- 稀疏矩阵操作优化
- 几何量的懒惰求值和缓存
- 支持大规模网格的分批处理

### 算法特定注意事项

**变分切割**:
- 使用水平集方法隐式表示边界
- 梯度下降优化，支持Armijo线搜索
- 多种能量项：长度正则化、失真度量、可见性

**纹理映射**:
- 需要合适的网格分辨率表示切割边界
- 锥点选择影响参数化质量
- BFF算法对边界条件敏感

**表面填充**:
- 空间填充曲线在UV空间生成，然后映射回3D
- 图案密度和网格分辨率需要匹配
- 支持用户自定义权重引导

## 常见问题和解决方案

### GUI相关问题

**ImGui中文字符显示为乱码 (????)**
- 问题：ImGui默认字体不支持中文字符
- 解决：在 `initializeImGui()` 函数中添加中文字体加载，使用系统字体如 `simhei.ttf`, `simsun.ttc`, `msyh.ttc`
- 或者：将UI文本改为英文以避免字体问题

**构建特定目标**
```bash
# 只构建ImGui GUI
cmake --build --preset stm-debug --target SurfaceTextureMapping_imgui_gui

# 只构建命令行工具
cmake --build --preset stm-debug --target SurfaceTextureMapping

# 只构建测试程序
cmake --build --preset stm-debug --target test_spot
```

### 常见工作流程

### 开发新算法
1. 在相应的 `projects/*/core/` 目录添加算法实现
2. 更新 `projects/*/gui/` 添加用户界面
3. 在 `projects/*/examples/` 添加测试用例
4. 更新CMakeLists.txt添加新的可执行文件

### 调试几何算法
- 使用GUI查看中间结果的可视化
- 输出网格到OBJ格式检查几何正确性
- 利用 `Timer` 类测量算法性能
- 检查稀疏矩阵的条件数和求解器收敛性

### 处理大规模网格
- 使用等各向性重网格化预处理
- 选择合适的稀疏矩阵求解器
- 监控内存使用和计算时间
- 考虑分批处理策略

### 重要注意事项

**原始Variational Cuts项目**：这是研究代码转储，作者明确警告代码不够完善和用户友好。主要用于学术研究和比较。

**Surface Texture Mapping项目**：这是现代化的完整实现，推荐用于实际应用。包含完整的纹理映射流水线，支持多种填充图案。

**算法参数调优**：
- Hencky能量权重建议从3.0开始
- 能量项都已无量纲化，权重值通常在10^0到10^3范围内
- 网格分辨率必须足够高以表示切割边界，否则会出现数值噪声

这个代码库是计算几何和数字制造领域的研究工具，结合了理论严谨性和工程实用性。开发时需要理解底层的数学原理和数值方法特性。