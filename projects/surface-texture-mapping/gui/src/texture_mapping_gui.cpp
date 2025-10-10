#include "texture_mapping_gui.h"

#include <nanogui/layout.h>
#include <nanogui/button.h>
#include <nanogui/textbox.h>
#include <nanogui/checkbox.h>
#include <nanogui/slider.h>
#include <nanogui/combobox.h>
#include <nanogui/label.h>
#include <nanogui/messagedialog.h>
#include <nanogui/tabwidget.h>
#include <nanogui/vscrollpanel.h>

#include <iostream>
#include <iomanip>
#include <sstream>
#include <filesystem>

namespace fs = std::filesystem;
using namespace nanogui;

namespace SurfaceTextureMapping {

TextureMappingGUI::TextureMappingGUI()
    : nanogui::Screen(nanogui::Vector2i(1600, 900), "曲面纹理映射可视化") {

    // 初始化核心组件
    meshProcessor_ = std::make_unique<MeshProcessor>();
    variationalCutter_ = std::make_unique<VariationalCutter>();
    textureMapper_ = std::make_unique<TextureMapper>();
    surfaceFiller_ = std::make_unique<SurfaceFiller>();

    // 初始化可视化组件
    viewer_ = std::make_unique<Viewer>();
}

TextureMappingGUI::~TextureMappingGUI() = default;

bool TextureMappingGUI::initialize() {
    // 创建GUI面板
    createMainPanel();
    createViewPanel();
    createStatsPanel();

    // 设置布局
    performLayout();

    // 默认加载spot.obj进行测试
    std::string defaultModel = "data/spot.obj";
    if (fs::exists(defaultModel)) {
        loadMesh(defaultModel);
    }

    return true;
}

void TextureMappingGUI::createMainPanel() {
    mainPanel_ = new Window(this, "控制面板");
    mainPanel_->setPosition(Vector2i(15, 15));
    mainPanel_->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 10));

    // 文件操作
    Widget* fileWidget = new Widget(mainPanel_);
    fileWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));

    new Label(fileWidget, "文件:");
    filePathTextBox_ = new TextBox(fileWidget, "data/spot.obj");
    filePathTextBox_->setFixedWidth(200);

    loadButton_ = new Button(fileWidget, "加载");
    loadButton_->setCallback([this]() {
        loadMesh(filePathTextBox_->value());
    });

    saveButton_ = new Button(fileWidget, "保存");
    saveButton_->setCallback([this]() {
        saveMesh("output.obj");
    });

    // 创建选项卡
    TabWidget* tabs = new TabWidget(mainPanel_);

    // 网格处理选项卡
    Widget* meshTab = tabs->createTab("网格处理");
    meshTab->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));
    createMeshProcessingControls(meshTab);

    // 变分切缝选项卡
    Widget* cuttingTab = tabs->createTab("变分切缝");
    cuttingTab->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));
    createCuttingControls(cuttingTab);

    // UV映射选项卡
    Widget* uvTab = tabs->createTab("UV映射");
    uvTab->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));
    createUVMappingControls(uvTab);

    // 表面填充选项卡
    Widget* fillingTab = tabs->createTab("表面填充");
    fillingTab->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));
    createFillingControls(fillingTab);

    // 视图选项选项卡
    Widget* viewTab = tabs->createTab("视图");
    viewTab->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 5, 5));
    createViewControls(viewTab);

    tabs->setActiveTab(0);
}

void TextureMappingGUI::createMeshProcessingControls(Widget* parent) {
    meshProcessingWidgets_.enableRemesh = new CheckBox(parent, "启用重网格化");
    meshProcessingWidgets_.enableRemesh->setChecked(true);

    Widget* edgeWidget = new Widget(parent);
    edgeWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(edgeWidget, "目标边长:");
    meshProcessingWidgets_.targetEdgeLength = new Slider(edgeWidget);
    meshProcessingWidgets_.targetEdgeLength->setValue(0.5f);
    meshProcessingWidgets_.targetEdgeLength->setRange({0.01f, 1.0f});
    meshProcessingWidgets_.edgeLengthText = new TextBox(edgeWidget, "0.02");
    meshProcessingWidgets_.edgeLengthText->setFixedWidth(60);

    meshProcessingWidgets_.targetEdgeLength->setCallback([this](float value) {
        meshProcessingWidgets_.edgeLengthText->setValue(formatNumber(value * 0.1, 3));
    });

    meshProcessingWidgets_.protectBoundary = new CheckBox(parent, "保护边界");
    meshProcessingWidgets_.protectBoundary->setChecked(true);

    meshProcessingWidgets_.applyButton = new Button(parent, "应用处理");
    meshProcessingWidgets_.applyButton->setCallback([this]() {
        processMesh();
    });
}

void TextureMappingGUI::createCuttingControls(Widget* parent) {
    cuttingWidgets_.enableCutting = new CheckBox(parent, "启用变分切缝");
    cuttingWidgets_.enableCutting->setChecked(true);

    Widget* lengthRegWidget = new Widget(parent);
    lengthRegWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(lengthRegWidget, "长度正则化:");
    cuttingWidgets_.lengthRegularization = new Slider(lengthRegWidget);
    cuttingWidgets_.lengthRegularization->setValue(0.1f);
    cuttingWidgets_.lengthRegularization->setRange({0.0f, 1.0f});

    Widget* smoothRegWidget = new Widget(parent);
    smoothRegWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(smoothRegWidget, "平滑正则化:");
    cuttingWidgets_.smoothRegularization = new Slider(smoothRegWidget);
    cuttingWidgets_.smoothRegularization->setValue(0.05f);
    cuttingWidgets_.smoothRegularization->setRange({0.0f, 1.0f});

    Widget* iterWidget = new Widget(parent);
    iterWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(iterWidget, "迭代次数:");
    cuttingWidgets_.iterations = new IntBox<int>(iterWidget);
    cuttingWidgets_.iterations->setValue(50);
    cuttingWidgets_.iterations->setEditable(true);
    cuttingWidgets_.iterations->setSpinnable(true);
    cuttingWidgets_.iterations->setMinMaxValues(1, 500);

    cuttingWidgets_.computeCutsButton = new Button(parent, "计算切缝");
    cuttingWidgets_.computeCutsButton->setCallback([this]() {
        computeCuts();
    });

    cuttingWidgets_.showCuts = new CheckBox(parent, "显示切缝");
    cuttingWidgets_.showCuts->setChecked(true);
    cuttingWidgets_.showCuts->setCallback([this](bool value) {
        updateVisualization();
    });
}

void TextureMappingGUI::createUVMappingControls(Widget* parent) {
    Widget* methodWidget = new Widget(parent);
    methodWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(methodWidget, "映射方法:");
    cuttingWidgets_.mappingMethod = new ComboBox(parent, {"共形映射", "面积保持", "混合模式"});
    cuttingWidgets_.mappingMethod->setSelectedIndex(0);

    uvMappingWidgets_.autoDetectCones = new CheckBox(parent, "自动检测锥点");
    uvMappingWidgets_.autoDetectCones->setChecked(true);

    Widget* thresholdWidget = new Widget(parent);
    thresholdWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(thresholdWidget, "曲率阈值:");
    uvMappingWidgets_.curvatureThreshold = new Slider(thresholdWidget);
    uvMappingWidgets_.curvatureThreshold->setValue(0.1f);
    uvMappingWidgets_.curvatureThreshold->setRange({0.01f, 1.0f});

    uvMappingWidgets_.computeUVButton = new Button(parent, "计算UV");
    uvMappingWidgets_.computeUVButton->setCallback([this]() {
        computeUV();
    });

    uvMappingWidgets_.showUV = new CheckBox(parent, "显示UV网格");
    uvMappingWidgets_.showUV->setChecked(false);
    uvMappingWidgets_.showUV->setCallback([this](bool value) {
        updateVisualization();
    });

    uvMappingWidgets_.showDistortion = new CheckBox(parent, "显示失真度");
    uvMappingWidgets_.showDistortion->setChecked(false);
    uvMappingWidgets_.showDistortion->setCallback([this](bool value) {
        updateVisualization();
    });
}

void TextureMappingGUI::createFillingControls(Widget* parent) {
    Widget* patternWidget = new Widget(parent);
    patternWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(patternWidget, "图案类型:");
    fillingWidgets_.patternType = new ComboBox(parent,
        {"网格", "六边形", "同心圆", "螺旋", "希尔伯特", "皮亚诺"});
    fillingWidgets_.patternType->setSelectedIndex(0);

    Widget* spacingWidget = new Widget(parent);
    spacingWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(spacingWidget, "图案间距:");
    fillingWidgets_.patternSpacing = new Slider(spacingWidget);
    fillingWidgets_.patternSpacing->setValue(0.05f);
    fillingWidgets_.patternSpacing->setRange({0.01f, 0.2f});

    fillingWidgets_.respectBoundary = new CheckBox(parent, "遵循边界");
    fillingWidgets_.respectBoundary->setChecked(true);

    Widget* depthWidget = new Widget(parent);
    depthWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(depthWidget, "递归深度:");
    fillingWidgets_.recursionDepth = new IntBox<int>(depthWidget);
    fillingWidgets_.recursionDepth->setValue(3);
    fillingWidgets_.recursionDepth->setEditable(true);
    fillingWidgets_.recursionDepth->setSpinnable(true);
    fillingWidgets_.recursionDepth->setMinMaxValues(1, 10);

    fillingWidgets_.generatePattern = new Button(parent, "生成图案");
    fillingWidgets_.generatePattern->setCallback([this]() {
        generatePattern();
    });

    fillingWidgets_.showPattern = new CheckBox(parent, "显示填充图案");
    fillingWidgets_.showPattern->setChecked(true);
    fillingWidgets_.showPattern->setCallback([this](bool value) {
        updateVisualization();
    });
}

void TextureMappingGUI::createViewControls(Widget* parent) {
    viewOptions_.showWireframe = new CheckBox(parent, "显示线框");
    viewOptions_.showWireframe->setChecked(false);
    viewOptions_.showWireframe->setCallback([this](bool value) {
        updateVisualization();
    });

    viewOptions_.showNormals = new CheckBox(parent, "显示法线");
    viewOptions_.showNormals->setChecked(false);
    viewOptions_.showNormals->setCallback([this](bool value) {
        updateVisualization();
    });

    viewOptions_.useFlatShading = new CheckBox(parent, "平面着色");
    viewOptions_.useFlatShading->setChecked(false);
    viewOptions_.useFlatShading->setCallback([this](bool value) {
        updateVisualization();
    });

    Widget* colorWidget = new Widget(parent);
    colorWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(colorWidget, "颜色模式:");
    viewOptions_.colorMode = new ComboBox(parent,
        {"单色", "顶点颜色", "失真热图", "UV坐标"});
    viewOptions_.colorMode->setSelectedIndex(0);

    Widget* pointSizeWidget = new Widget(parent);
    pointSizeWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(pointSizeWidget, "点大小:");
    viewOptions_.pointSize = new Slider(pointSizeWidget);
    viewOptions_.pointSize->setValue(0.5f);
    viewOptions_.pointSize->setRange({0.1f, 5.0f});

    Widget* lineWidthWidget = new Widget(parent);
    lineWidthWidget->setLayout(new BoxLayout(Orientation::Horizontal, Alignment::Middle, 0, 5));
    new Label(lineWidthWidget, "线宽:");
    viewOptions_.lineWidth = new Slider(lineWidthWidget);
    viewOptions_.lineWidth->setValue(0.5f);
    viewOptions_.lineWidth->setRange({0.1f, 5.0f});
}

void TextureMappingGUI::createViewPanel() {
    viewPanel_ = new Window(this, "3D视图");
    viewPanel_->setPosition(Vector2i(400, 15));
    viewPanel_->setSize(Vector2i(800, 600));
    viewPanel_->setLayout(new BoxLayout(Orientation::Vertical));

    // 视图区域占位符
    Widget* viewArea = new Widget(viewPanel_);
    viewArea->setFixedSize(Vector2i(780, 560));
}

void TextureMappingGUI::createStatsPanel() {
    statsPanel_ = new Window(this, "统计信息");
    statsPanel_->setPosition(Vector2i(1220, 15));
    statsPanel_->setLayout(new BoxLayout(Orientation::Vertical, Alignment::Fill, 10, 10));

    // 网格统计
    new Label(statsPanel_, "网格信息", "sans-bold");
    statsLabels_.vertexCount = new Label(statsPanel_, "顶点数: -");
    statsLabels_.faceCount = new Label(statsPanel_, "面数: -");
    statsLabels_.edgeCount = new Label(statsPanel_, "边数: -");
    statsLabels_.avgEdgeLength = new Label(statsPanel_, "平均边长: -");
    statsLabels_.isManifold = new Label(statsPanel_, "流形: -");

    new Label(statsPanel_, "", "sans"); // 分隔符

    // 切缝统计
    new Label(statsPanel_, "切缝信息", "sans-bold");
    statsLabels_.cutCount = new Label(statsPanel_, "切缝数: -");

    new Label(statsPanel_, "", "sans"); // 分隔符

    // UV统计
    new Label(statsPanel_, "UV信息", "sans-bold");
    statsLabels_.distortion = new Label(statsPanel_, "失真度: -");

    new Label(statsPanel_, "", "sans"); // 分隔符

    // 填充统计
    new Label(statsPanel_, "填充信息", "sans-bold");
    statsLabels_.coverage = new Label(statsPanel_, "覆盖率: -");
}

void TextureMappingGUI::loadMesh(const std::string& filepath) {
    try {
        if (!meshProcessor_->loadMesh(filepath)) {
            showError("加载错误", "无法加载网格文件: " + filepath);
            return;
        }

        meshLoaded_ = true;
        meshProcessed_ = false;
        cutsComputed_ = false;
        uvComputed_ = false;
        patternGenerated_ = false;

        filePathTextBox_->setValue(filepath);
        updateStatistics();
        updateVisualization();

        showMessage("成功", "成功加载网格: " + filepath);
    } catch (const std::exception& e) {
        showError("加载错误", std::string("加载网格时出错: ") + e.what());
    }
}

void TextureMappingGUI::saveMesh(const std::string& filepath) {
    if (!meshLoaded_) {
        showError("保存错误", "没有加载的网格");
        return;
    }

    try {
        if (!meshProcessor_->saveMesh(filepath)) {
            showError("保存错误", "无法保存网格文件");
            return;
        }

        // 如果有UV映射，导出带UV的OBJ
        if (uvComputed_ && currentUVMapping_.has_value()) {
            std::string uvFile = filepath.substr(0, filepath.find_last_of('.')) + "_uv.obj";
            textureMapper_->exportUVMesh(uvFile, currentUVMapping_.value());
        }

        // 如果有填充图案，导出路径
        if (patternGenerated_ && currentFilling_.has_value()) {
            std::string pathFile = filepath.substr(0, filepath.find_last_of('.')) + "_paths.obj";
            surfaceFiller_->exportPaths3D(currentFilling_.value().paths3D, pathFile);

            std::string svgFile = filepath.substr(0, filepath.find_last_of('.')) + "_pattern.svg";
            surfaceFiller_->exportPathsToSVG(currentFilling_.value().pathsUV, svgFile);
        }

        showMessage("成功", "成功保存到: " + filepath);
    } catch (const std::exception& e) {
        showError("保存错误", std::string("保存网格时出错: ") + e.what());
    }
}

void TextureMappingGUI::processMesh() {
    if (!meshLoaded_) {
        showError("处理错误", "请先加载网格");
        return;
    }

    try {
        // 基础修复
        meshProcessor_->basicRepair();

        // 重网格化
        if (meshProcessingWidgets_.enableRemesh->checked()) {
            float targetLength = meshProcessingWidgets_.targetEdgeLength->value() * 0.1;
            bool protectBoundary = meshProcessingWidgets_.protectBoundary->checked();
            meshProcessor_->isotropicRemeshing(targetLength, 3, protectBoundary);
        }

        // 流形化
        meshProcessor_->makeManifold();

        meshProcessed_ = true;
        updateStatistics();
        updateVisualization();

        showMessage("成功", "网格处理完成");
    } catch (const std::exception& e) {
        showError("处理错误", std::string("处理网格时出错: ") + e.what());
    }
}

void TextureMappingGUI::computeCuts() {
    if (!meshLoaded_) {
        showError("切缝错误", "请先加载网格");
        return;
    }

    if (!cuttingWidgets_.enableCutting->checked()) {
        return;
    }

    try {
        variationalCutter_->setMesh(meshProcessor_->getMesh(),
                                   meshProcessor_->getGeometry());

        VariationalCutter::CuttingParams params;
        params.lengthRegularization = cuttingWidgets_.lengthRegularization->value();
        params.smoothRegularization = cuttingWidgets_.smoothRegularization->value();
        params.maxIterations = cuttingWidgets_.iterations->value();
        params.timeStep = 0.01;

        currentCuts_ = variationalCutter_->computeOptimalCuts(params);

        cutsComputed_ = true;
        updateStatistics();
        updateVisualization();

        showMessage("成功", "变分切缝计算完成");
    } catch (const std::exception& e) {
        showError("切缝错误", std::string("计算切缝时出错: ") + e.what());
    }
}

void TextureMappingGUI::computeUV() {
    if (!meshLoaded_) {
        showError("UV错误", "请先加载网格");
        return;
    }

    try {
        auto mesh = meshProcessor_->getMesh();
        auto geometry = meshProcessor_->getGeometry();

        // 如果计算了切缝，应用切缝
        if (cutsComputed_ && !currentCuts_.empty()) {
            auto cutMesh = variationalCutter_->applyCutsToMesh(currentCuts_);
            if (cutMesh) {
                mesh = cutMesh;
            }
        }

        textureMapper_->setMesh(mesh, geometry);

        TextureMapper::MappingParams params;
        params.automaticConeDetection = uvMappingWidgets_.autoDetectCones->checked();
        params.curvatureThreshold = uvMappingWidgets_.curvatureThreshold->value();

        currentUVMapping_ = textureMapper_->computeUVMapping(params);

        if (!currentUVMapping_.has_value()) {
            showError("UV错误", "UV参数化失败");
            return;
        }

        uvComputed_ = true;
        updateStatistics();
        updateVisualization();

        showMessage("成功", "UV映射计算完成");
    } catch (const std::exception& e) {
        showError("UV错误", std::string("计算UV时出错: ") + e.what());
    }
}

void TextureMappingGUI::generatePattern() {
    if (!uvComputed_ || !currentUVMapping_.has_value()) {
        showError("填充错误", "请先计算UV映射");
        return;
    }

    try {
        surfaceFiller_->setInput(meshProcessor_->getMesh(),
                                meshProcessor_->getGeometry(),
                                currentUVMapping_.value());

        SurfaceFiller::FillingParams params;

        // 设置图案类型
        int patternIndex = fillingWidgets_.patternType->selectedIndex();
        params.type = static_cast<SurfaceFiller::PatternType>(patternIndex);

        params.spacing = fillingWidgets_.patternSpacing->value();
        params.respectBoundary = fillingWidgets_.respectBoundary->checked();
        params.recursionDepth = fillingWidgets_.recursionDepth->value();

        currentFilling_ = surfaceFiller_->generateFilling(params);

        patternGenerated_ = true;
        updateStatistics();
        updateVisualization();

        showMessage("成功", "填充图案生成完成");
    } catch (const std::exception& e) {
        showError("填充错误", std::string("生成图案时出错: ") + e.what());
    }
}

void TextureMappingGUI::updateVisualization() {
    // 这里更新3D视图的显示内容
    // 根据各种复选框的状态显示不同的内容
}

void TextureMappingGUI::updateStatistics() {
    if (meshLoaded_) {
        auto stats = meshProcessor_->getMeshStats();
        statsLabels_.vertexCount->setCaption("顶点数: " + std::to_string(stats.numVertices));
        statsLabels_.faceCount->setCaption("面数: " + std::to_string(stats.numFaces));
        statsLabels_.edgeCount->setCaption("边数: " + std::to_string(stats.numEdges));
        statsLabels_.avgEdgeLength->setCaption("平均边长: " + formatNumber(stats.avgEdgeLength, 4));
        statsLabels_.isManifold->setCaption("流形: " + std::string(stats.isManifold ? "是" : "否"));
    }

    if (cutsComputed_ && !currentCuts_.empty()) {
        statsLabels_.cutCount->setCaption("切缝数: " + std::to_string(currentCuts_.size()));
    }

    if (uvComputed_ && currentUVMapping_.has_value()) {
        auto& mapping = currentUVMapping_.value();
        statsLabels_.distortion->setCaption("失真度: " + formatNumber(mapping.totalDistortion, 4));
    }

    if (patternGenerated_ && currentFilling_.has_value()) {
        auto& filling = currentFilling_.value();
        statsLabels_.coverage->setCaption("覆盖率: " + formatNumber(filling.coverage * 100, 2) + "%");
    }
}

void TextureMappingGUI::showMessage(const std::string& title, const std::string& message) {
    auto dlg = new MessageDialog(this, MessageDialog::Type::Information,
                                title, message);
    dlg->setCallback([](int result) {});
}

void TextureMappingGUI::showError(const std::string& title, const std::string& error) {
    auto dlg = new MessageDialog(this, MessageDialog::Type::Warning,
                                title, error);
    dlg->setCallback([](int result) {});
}

std::string TextureMappingGUI::formatNumber(double value, int precision) const {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(precision) << value;
    return ss.str();
}

glm::vec3 TextureMappingGUI::eigenToGLM(const geometrycentral::Vector3& v) const {
    return glm::vec3(v.x, v.y, v.z);
}

void TextureMappingGUI::drawContents() {
    // 绘制3D视图内容
    if (viewer_) {
        // viewer_->draw();
    }
}

bool TextureMappingGUI::keyboardEvent(int key, int scancode, int action, int modifiers) {
    if (Screen::keyboardEvent(key, scancode, action, modifiers))
        return true;

    // 处理快捷键
    if (action == GLFW_PRESS) {
        if (key == GLFW_KEY_O && modifiers & GLFW_MOD_CONTROL) {
            // Ctrl+O: 打开文件
            loadMesh(filePathTextBox_->value());
            return true;
        } else if (key == GLFW_KEY_S && modifiers & GLFW_MOD_CONTROL) {
            // Ctrl+S: 保存文件
            saveMesh("output.obj");
            return true;
        }
    }

    return false;
}

bool TextureMappingGUI::mouseButtonEvent(const Vector2i& p, int button, bool down, int modifiers) {
    if (Screen::mouseButtonEvent(p, button, down, modifiers))
        return true;

    // 传递给3D视图
    if (viewer_) {
        // viewer_->mouseButtonEvent(p, button, down, modifiers);
    }

    return false;
}

bool TextureMappingGUI::mouseMotionEvent(const Vector2i& p, const Vector2i& rel, int button, int modifiers) {
    if (Screen::mouseMotionEvent(p, rel, button, modifiers))
        return true;

    // 传递给3D视图
    if (viewer_) {
        // viewer_->mouseMotionEvent(p, rel, button, modifiers);
    }

    return false;
}

bool TextureMappingGUI::scrollEvent(const Vector2i& p, const Vector2f& rel) {
    if (Screen::scrollEvent(p, rel))
        return true;

    // 传递给3D视图
    if (viewer_) {
        // viewer_->scrollEvent(p, rel);
    }

    return false;
}

} // namespace SurfaceTextureMapping