# Surface Texture Mapping GUI Library
cmake_minimum_required(VERSION 3.10)

# 收集GUI源文件（排除ImGui相关文件，它们直接在可执行文件中编译）
file(GLOB STM_GUI_SRCS src/gui_main.cpp src/texture_mapping_gui.cpp)
file(GLOB_RECURSE STM_GUI_HEADERS include/*.h include/*.hpp)
# 排除ImGui相关文件
list(FILTER STM_GUI_SRCS EXCLUDE REGEX ".*imgui.*")
list(FILTER STM_GUI_HEADERS EXCLUDE REGEX ".*imgui.*")

# 创建GUI库
add_library(
    stm-gui
    ${STM_GUI_SRCS}
    ${STM_GUI_HEADERS}
)

# 设置包含目录
target_include_directories(stm-gui PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../deps/nanogui/ext/nanovg/src  # nanovg头文件路径
)

# 链接依赖库
target_link_libraries(stm-gui
    # viewer       # 暂时移除依赖以解决OpenGL头文件冲突
    stm-core     # 依赖STM核心库
    core         # 依赖主项目的core库
    nanogui
    ${OPENGL_LIBRARIES}
    ${GLEW_LIBRARIES}
)

# 设置C++17标准
target_compile_features(stm-gui PUBLIC cxx_std_17)

# 编译选项
if(MSVC)
    target_compile_options(stm-gui PRIVATE /W4)
else()
    target_compile_options(stm-gui PRIVATE -Wall -Wextra -pedantic)
endif()