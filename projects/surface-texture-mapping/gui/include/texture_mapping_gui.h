#pragma once

#include <memory>
#include <string>
#include <vector>
#include <nanogui/nanogui.h>

#include "mesh_processing.h"
#include "variational_cutting.h"
#include "texture_mapping.h"
#include "surface_filling.h"

// 暂时移除对主项目viewer的依赖以解决OpenGL头文件冲突
// #include "../../gui/include/viewer.h"
// #include "../../gui/include/scene_mesh.h"
// #include "../../gui/include/scene_lines.h"

namespace SurfaceTextureMapping {

/**
 * 纹理映射GUI界面
 * 提供交互式的可视化和参数调整
 */
class TextureMappingGUI : public nanogui::Screen {
public:
    TextureMappingGUI();
    ~TextureMappingGUI();

    /**
     * 初始化GUI
     */
    bool initialize();

    /**
     * 绘制回调
     */
    virtual void drawContents() override;

    /**
     * 键盘事件处理
     */
    virtual bool keyboardEvent(int key, int scancode, int action, int modifiers) override;

    /**
     * 鼠标事件处理
     */
    virtual bool mouseButtonEvent(const nanogui::Vector2i& p, int button, bool down, int modifiers) override;
    virtual bool mouseMotionEvent(const nanogui::Vector2i& p, const nanogui::Vector2i& rel, int button, int modifiers) override;
    virtual bool scrollEvent(const nanogui::Vector2i& p, const nanogui::Vector2f& rel) override;

private:
    // GUI组件
    nanogui::Window* mainPanel_;
    nanogui::Window* viewPanel_;
    nanogui::Window* statsPanel_;

    // 文件选择
    nanogui::TextBox* filePathTextBox_;
    nanogui::Button* loadButton_;
    nanogui::Button* saveButton_;

    // 网格预处理控件
    struct {
        nanogui::CheckBox* enableRemesh;
        nanogui::Slider* targetEdgeLength;
        nanogui::TextBox* edgeLengthText;
        nanogui::CheckBox* protectBoundary;
        nanogui::Button* applyButton;
    } meshProcessingWidgets_;

    // 变分切缝控件
    struct {
        nanogui::CheckBox* enableCutting;
        nanogui::Slider* lengthRegularization;
        nanogui::Slider* smoothRegularization;
        nanogui::IntBox<int>* iterations;
        nanogui::Button* computeCutsButton;
        nanogui::CheckBox* showCuts;
    } cuttingWidgets_;

    // UV映射控件
    struct {
        nanogui::ComboBox* mappingMethod;
        nanogui::CheckBox* autoDetectCones;
        nanogui::Slider* curvatureThreshold;
        nanogui::Button* computeUVButton;
        nanogui::CheckBox* showUV;
        nanogui::CheckBox* showDistortion;
    } uvMappingWidgets_;

    // 表面填充控件
    struct {
        nanogui::ComboBox* patternType;
        nanogui::Slider* patternSpacing;
        nanogui::CheckBox* respectBoundary;
        nanogui::IntBox<int>* recursionDepth;
        nanogui::Button* generatePattern;
        nanogui::CheckBox* showPattern;
    } fillingWidgets_;

    // 可视化选项
    struct {
        nanogui::CheckBox* showWireframe;
        nanogui::CheckBox* showNormals;
        nanogui::CheckBox* useFlatShading;
        nanogui::ComboBox* colorMode;
        nanogui::Slider* pointSize;
        nanogui::Slider* lineWidth;
    } viewOptions_;

    // 统计信息标签
    struct {
        nanogui::Label* vertexCount;
        nanogui::Label* faceCount;
        nanogui::Label* edgeCount;
        nanogui::Label* avgEdgeLength;
        nanogui::Label* isManifold;
        nanogui::Label* cutCount;
        nanogui::Label* distortion;
        nanogui::Label* coverage;
    } statsLabels_;

    // 核心组件
    std::unique_ptr<MeshProcessor> meshProcessor_;
    std::unique_ptr<VariationalCutter> variationalCutter_;
    std::unique_ptr<TextureMapper> textureMapper_;
    std::unique_ptr<SurfaceFiller> surfaceFiller_;

    // 可视化组件
    std::unique_ptr<Viewer> viewer_;
    std::shared_ptr<SceneMesh> originalMesh_;
    std::shared_ptr<SceneMesh> processedMesh_;
    std::shared_ptr<SceneLines> cutLines_;
    std::shared_ptr<SceneLines> patternLines_;

    // 当前数据
    std::vector<VariationalCutter::CutCurve> currentCuts_;
    std::optional<TextureMapper::UVMapping> currentUVMapping_;
    std::optional<SurfaceFiller::FillingResult> currentFilling_;

    // 状态标志
    bool meshLoaded_ = false;
    bool meshProcessed_ = false;
    bool cutsComputed_ = false;
    bool uvComputed_ = false;
    bool patternGenerated_ = false;

    // GUI创建函数
    void createMainPanel();
    void createViewPanel();
    void createStatsPanel();
    void createMeshProcessingControls(nanogui::Widget* parent);
    void createCuttingControls(nanogui::Widget* parent);
    void createUVMappingControls(nanogui::Widget* parent);
    void createFillingControls(nanogui::Widget* parent);
    void createViewControls(nanogui::Widget* parent);

    // 功能函数
    void loadMesh(const std::string& filepath);
    void saveMesh(const std::string& filepath);
    void processMesh();
    void computeCuts();
    void computeUV();
    void generatePattern();
    void updateVisualization();
    void updateStatistics();

    // 辅助函数
    void showMessage(const std::string& title, const std::string& message);
    void showError(const std::string& title, const std::string& error);
    std::string formatNumber(double value, int precision = 2) const;
    glm::vec3 eigenToGLM(const geometrycentral::Vector3& v) const;
};

} // namespace SurfaceTextureMapping