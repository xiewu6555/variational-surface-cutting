# Cuts GUI 模块说明

## 概述

cuts-gui 是 Variational Surface Cutting 项目的图形用户界面模块，提供了变分切缝算法的可视化和交互功能。

## 架构设计

### 模块依赖关系

```
cuts.exe (可执行文件)
    ├── cuts-gui (GUI库)
    │   ├── viewer (通用3D渲染框架)
    │   ├── cuts-core (变分切缝算法)
    │   └── core (几何基础库)
    └── viewer (渲染框架)
```

### 主要组件

1. **Gui类** (`gui.h/cpp`)
   - 管理整个用户界面
   - 协调viewer和scene对象
   - 处理用户交互事件

2. **Viewer类** (来自viewer库)
   - OpenGL渲染管线
   - NanoGUI窗口管理
   - 相机控制和视角操作

3. **ProjectGui类** (`project_gui.h`)
   - 项目特定的GUI扩展接口
   - 允许算法模块添加自定义控件

4. **SceneMesh类** (来自viewer库)
   - 3D网格渲染
   - 支持边、面、点的可视化
   - 着色器管理

## 运行方式

### 1. 直接运行（无参数）

```bash
cuts.exe
```

启动GUI，需要通过界面加载网格文件。

### 2. 命令行加载模型

```bash
cuts.exe path/to/model.obj
```

启动GUI并自动加载指定的OBJ模型文件。

### 3. 使用批处理脚本

```batch
run_cuts.bat              # 直接启动
run_cuts.bat model.obj    # 加载指定模型
```

或者直接拖拽OBJ文件到批处理文件上。

## GUI功能

### 基础渲染选项
- **Show Edges**: 显示/隐藏网格边
- **Edge Opacity**: 调整边的透明度
- **Show Dual**: 显示对偶网格
- **Ground Plane**: 显示地平面

### 相机控制
- 左键拖拽：旋转视角
- 右键拖拽：平移视角
- 滚轮：缩放
- 双击：居中对象

### 变分切缝功能
- 初始化切缝
- 优化参数调节
- 实时预览切缝
- 导出切缝结果

## 代码结构

```
projects/cuts/gui/
├── include/
│   └── cuts_gui.h          # Cuts特定的GUI扩展
├── src/
│   └── cuts_gui.cpp        # GUI实现
└── CMakeLists.txt          # 构建配置
```

## 扩展开发

### 添加新的GUI控件

```cpp
// 在cuts_gui.cpp中
void CutsGui::buildUserInterface() {
    // 添加新的控件
    nanogui::Button* myButton = new nanogui::Button(window, "My Button");
    myButton->setCallback([]() {
        std::cout << "Button clicked!" << std::endl;
    });
}
```

### 自定义渲染

```cpp
// 自定义场景对象
class MySceneObject : public SceneObject {
    void draw() override {
        // OpenGL渲染代码
    }
};
```

## 构建要求

- C++11或更高版本
- OpenGL 3.3+
- 依赖库：
  - NanoGUI (GUI框架)
  - GLFW (窗口管理)
  - GLEW/GLAD (OpenGL加载)
  - Eigen3 (线性代数)

## 常见问题

### Q: GUI启动失败
A: 检查OpenGL驱动是否正确安装，确保支持OpenGL 3.3或更高版本。

### Q: 无法加载大型网格
A: 调整内存设置或使用网格简化工具预处理。

### Q: 渲染性能低
A:
- 启用Release模式构建
- 减少渲染的边数量
- 使用简化的着色器

## 调试技巧

1. **启用控制台输出**
   ```cpp
   std::cout << "Debug info" << std::endl;
   ```

2. **OpenGL错误检查**
   ```cpp
   GLenum error = glGetError();
   if (error != GL_NO_ERROR) {
       std::cerr << "OpenGL Error: " << error << std::endl;
   }
   ```

3. **性能分析**
   - 使用内置的FPS计数器
   - 启用渲染统计信息

## 相关文档

- [主项目README](../../../README.md)
- [Core库文档](../core/README.md)
- [Viewer框架文档](../../../gui/README.md)