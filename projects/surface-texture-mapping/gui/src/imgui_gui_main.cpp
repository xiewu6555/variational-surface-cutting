/**
 * ImGui版本的Surface Texture Mapping GUI主程序
 * 提供独立的3D可视化和交互界面
 */

#include <iostream>
#include <memory>
#include <filesystem>

#include "imgui_texture_mapping_gui.h"

namespace fs = std::filesystem;

/**
 * 查找并设置工作目录为项目根目录
 * 从可执行文件路径向上查找，直到找到包含"data"目录的目录
 */
bool setWorkingDirectoryToProjectRoot() {
    try {
        // 获取可执行文件的绝对路径
        fs::path exePath = fs::current_path();

        std::cout << "Current working directory: " << exePath << std::endl;

        // 从当前目录开始向上查找项目根目录
        // 项目根目录的标志是包含"data"子目录
        fs::path searchPath = exePath;
        int maxLevels = 10; // 最多向上查找10层

        for (int i = 0; i < maxLevels; ++i) {
            fs::path dataDir = searchPath / "data";

            if (fs::exists(dataDir) && fs::is_directory(dataDir)) {
                // 找到了包含data目录的路径，设置为工作目录
                fs::current_path(searchPath);
                std::cout << "Found project root: " << searchPath << std::endl;
                std::cout << "Working directory set to: " << fs::current_path() << std::endl;
                return true;
            }

            // 向上一级目录
            fs::path parent = searchPath.parent_path();
            if (parent == searchPath) {
                // 已经到达根目录
                break;
            }
            searchPath = parent;
        }

        std::cerr << "Warning: Could not find project root directory (containing 'data' folder)" << std::endl;
        std::cerr << "Working directory remains: " << fs::current_path() << std::endl;
        return false;

    } catch (const std::exception& e) {
        std::cerr << "Error setting working directory: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char** argv) {
    std::cout << "=== Surface Texture Mapping GUI - ImGui Version ===\n";
    std::cout << "Starting GUI application...\n\n";

    // 设置工作目录为项目根目录，使相对路径能够正确工作
    setWorkingDirectoryToProjectRoot();
    std::cout << std::endl;

    try {
        // 创建GUI实例
        auto gui = std::make_unique<SurfaceTextureMapping::ImGuiTextureMappingGUI>();

        // 初始化GUI系统
        if (!gui->initialize()) {
            std::cerr << "Failed to initialize GUI\n";
            return -1;
        }

        std::cout << "GUI initialized successfully\n";
        std::cout << "Controls:\n";
        std::cout << "  - Mouse: Rotate camera\n";
        std::cout << "  - Mouse wheel: Zoom in/out\n";
        std::cout << "  - ESC: Exit\n";
        std::cout << "  - Use Control Panel to load mesh and adjust parameters\n\n";

        // 如果提供了命令行参数，尝试加载网格文件
        if (argc > 1) {
            std::string meshPath = argv[1];
            std::cout << "Auto-loading mesh from command line: " << meshPath << "\n";
            gui->loadMesh(meshPath);
        }

        // 运行主循环
        gui->run();

        std::cout << "GUI closed successfully\n";
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return -1;
    } catch (...) {
        std::cerr << "Unknown error occurred\n";
        return -1;
    }
}