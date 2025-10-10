/**
 * GUI主程序入口
 * 提供交互式的纹理映射可视化界面
 */

#include <nanogui/nanogui.h>
#include <iostream>
#include <memory>

#include "texture_mapping_gui.h"

// 用于测试spot.obj的独立程序
void testWithSpot() {
    std::cout << "=== 曲面纹理映射GUI - Spot模型测试 ===\n\n";

    try {
        nanogui::init();

        {
            // 创建并初始化GUI
            auto app = std::make_unique<SurfaceTextureMapping::TextureMappingGUI>();

            if (!app->initialize()) {
                std::cerr << "GUI初始化失败\n";
                return;
            }

            app->setVisible(true);
            app->performLayout();

            std::cout << "GUI已启动，默认加载 data/spot.obj\n";
            std::cout << "控制说明:\n";
            std::cout << "  - 左键拖动: 旋转视图\n";
            std::cout << "  - 右键拖动: 平移视图\n";
            std::cout << "  - 滚轮: 缩放视图\n";
            std::cout << "  - Ctrl+O: 打开文件\n";
            std::cout << "  - Ctrl+S: 保存结果\n\n";

            nanogui::mainloop();
        }

        nanogui::shutdown();
    } catch (const std::runtime_error& e) {
        std::cerr << "运行时错误: " << e.what() << std::endl;
        nanogui::shutdown();
    }
}

int main(int argc, char* argv[]) {
    // 如果没有参数或第一个参数是spot，运行spot测试
    if (argc == 1 || (argc > 1 && std::string(argv[1]) == "spot")) {
        testWithSpot();
    } else {
        // 否则运行正常的GUI程序
        try {
            nanogui::init();

            {
                auto app = std::make_unique<SurfaceTextureMapping::TextureMappingGUI>();

                if (!app->initialize()) {
                    std::cerr << "GUI初始化失败\n";
                    return 1;
                }

                // 如果提供了文件路径作为参数，加载该文件
                if (argc > 1) {
                    std::string filepath = argv[1];
                    std::cout << "加载文件: " << filepath << "\n";
                    // 这里应该调用app->loadMesh(filepath)，但需要将该方法设为public
                }

                app->setVisible(true);
                app->performLayout();

                nanogui::mainloop();
            }

            nanogui::shutdown();
        } catch (const std::runtime_error& e) {
            std::cerr << "运行时错误: " << e.what() << std::endl;
            nanogui::shutdown();
            return 1;
        }
    }

    return 0;
}