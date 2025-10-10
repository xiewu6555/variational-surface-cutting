/**
 * UV失真可视化查看器 - 主程序入口
 *
 * 用法:
 *   DistortionViewer.exe [mesh_with_uv.obj]
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <iostream>
#include <string>
#include "imgui_distortion_viewer.h"

using namespace SurfaceTextureMapping;

void printUsage(const char* programName) {
    std::cout << "UV Distortion Viewer - Interactive distortion analysis tool\n\n";
    std::cout << "Usage:\n";
    std::cout << "  " << programName << " [options] [mesh_file.obj]\n\n";
    std::cout << "Options:\n";
    std::cout << "  -h, --help     Show this help message\n\n";
    std::cout << "Controls:\n";
    std::cout << "  Left Mouse    Rotate 3D view\n";
    std::cout << "  Scroll        Zoom in/out\n";
    std::cout << "  R             Reset camera\n";
    std::cout << "  ESC           Exit\n\n";
    std::cout << "Note: Mesh file must contain UV coordinates\n";
}

int main(int argc, char** argv) {
    std::cout << "========================================\n";
    std::cout << "UV Distortion Viewer\n";
    std::cout << "Version 1.0 (2025-09-30)\n";
    std::cout << "========================================\n\n";

    // 解析命令行参数
    std::string meshFile;
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            meshFile = arg;
        }
    }

    // 创建查看器
    ImGuiDistortionViewer viewer;

    // 初始化
    if (!viewer.initialize()) {
        std::cerr << "Failed to initialize distortion viewer" << std::endl;
        return 1;
    }

    // 如果指定了文件，尝试加载
    if (!meshFile.empty()) {
        std::cout << "Loading mesh: " << meshFile << std::endl;
        if (!viewer.loadMeshWithUV(meshFile)) {
            std::cerr << "Warning: Failed to load mesh file" << std::endl;
            std::cerr << "You can load a mesh using File menu" << std::endl;
        }
    } else {
        std::cout << "No mesh file specified. Use File > Load Mesh to load a mesh." << std::endl;
    }

    // 运行主循环
    viewer.run();

    // 清理
    viewer.cleanup();

    std::cout << "Distortion viewer closed successfully" << std::endl;
    return 0;
}