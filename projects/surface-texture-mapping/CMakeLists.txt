# 曲面纹理映射项目 - 模块化架构
cmake_minimum_required(VERSION 3.10)

project(SurfaceTextureMapping)

# C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 全局包含路径（保持与主项目一致）
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/core/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/gui/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps/geometry-central/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps/libigl/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps/cgal/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps/clipper2/CPP/Clipper2Lib/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps/nanogui/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../deps/glm)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../gui/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../../core/include)

# 查找依赖包
find_package(Eigen3 CONFIG REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Threads REQUIRED)

# 查找SuiteSparse/CHOLMOD
find_package(CHOLMOD CONFIG REQUIRED)

# GLFW for ImGui
find_package(glfw3 REQUIRED)

# ImGui (通过vcpkg提供)
find_package(imgui CONFIG REQUIRED)

# CGAL (通过vcpkg提供)
find_package(CGAL CONFIG)

# NanoGUI选项（仅用于原有GUI系统，对ImGui GUI禁用）
set(NANOGUI_BUILD_EXAMPLES OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL OFF CACHE BOOL " " FORCE)

# 添加依赖库子目录（如果尚未添加）
if(NOT TARGET geometry-central)
    add_subdirectory(../../deps/geometry-central geometry-central)
endif()
if(NOT TARGET Clipper2)
    add_subdirectory(../../deps/clipper2/CPP clipper2)
endif()

# 添加BFF库（Boundary First Flattening）
# 使用vcpkg提供的CHOLMOD
set(BFF_HAS_SUITESPARSE ON)
set(BFF_USE_SUITESPARSE ON CACHE BOOL "" FORCE)

if(NOT TARGET bff)
    # BFF需要SuiteSparse
    set(BFF_BUILD_CLI OFF CACHE BOOL "" FORCE)
    set(BFF_BUILD_GUI OFF CACHE BOOL "" FORCE)
    add_subdirectory(../../deps/boundary-first-flattening bff)
endif()
# 暂时禁用nanogui以避免与ImGui的冲突
# if(NOT TARGET nanogui)
#     add_subdirectory(../../deps/nanogui nanogui)
# endif()

# 确保使用vcpkg的GLFW，而不是nanogui的
find_package(glfw3 CONFIG REQUIRED)

# 添加库搜索路径以解决cholmod链接问题
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    link_directories(${CMAKE_PREFIX_PATH}/debug/lib)
else()
    link_directories(${CMAKE_PREFIX_PATH}/lib)
endif()

# 添加子模块
add_subdirectory(core)  # stm-core库

# 添加核心几何库依赖
if(NOT TARGET core)
    add_subdirectory(../../core core-lib)
endif()

# 真实算法库依赖
# 真实算法库依赖
if(NOT TARGET cuts-core)
    add_subdirectory(../../projects/cuts/core cuts-core)
endif()
if(NOT TARGET flatten-core)
    add_subdirectory(../../projects/flatten/core flatten-core)
endif()
# BFF已经包含在flatten-core中，不需要单独添加

# add_subdirectory(gui)   # stm-gui库 - 暂时禁用以避免nanogui冲突

# 命令行可执行文件
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME}
    stm-core
    ${OPENGL_LIBRARIES}
    Eigen3::Eigen
)

# GUI可执行文件（使用模块化架构）- 暂时禁用
# add_executable(${PROJECT_NAME}_gui gui/src/gui_main.cpp)
# target_link_libraries(${PROJECT_NAME}_gui
#     stm-gui
#     stm-core
#     # viewer  # 暂时移除依赖以解决OpenGL头文件冲突
#     ${CMAKE_THREAD_LIBS_INIT}
# )

# ImGui GUI可执行文件（集成真实算法）
add_executable(${PROJECT_NAME}_imgui_gui
    gui/src/imgui_gui_main.cpp
    gui/src/imgui_texture_mapping_gui.cpp  # 使用完整实现版本
    gui/src/glfw_stubs.cpp  # 解决GLFW版本兼容性问题
    # gui/src/cgal_mesh_processing.cpp  # CGAL网格处理实现 - 暂时禁用，待CGAL配置完成
    # gui/src/real_algorithms.cpp  # 真实算法流水线实现 - 已整合到主GUI类中
)

# 设置独立的包含目录，避免与nanogui冲突
target_include_directories(${PROJECT_NAME}_imgui_gui PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/core/include
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../core/include  # TODO: 暂时禁用以避免Vector3冲突
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../projects/cuts/core/include    # TODO: 暂时禁用EulerianShapeOptimizer
    # ${CMAKE_CURRENT_SOURCE_DIR}/../../projects/flatten/core/include # TODO: 暂时禁用BoundaryFirstFlattening
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/geometry-central/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/glm
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/cgal/include  # CGAL支持
)

# 链接库，包含真实算法
target_link_libraries(${PROJECT_NAME}_imgui_gui
    stm-core          # 核心纹理映射库
    core              # 添加核心几何库依赖
    cuts-core         # 变分切割算法
    flatten-core      # BFF展平算法
    geometry-central  # 几何中心库
    Eigen3::Eigen
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    glfw  # 使用vcpkg提供的GLFW
    imgui::imgui
    ${CMAKE_THREAD_LIBS_INIT}
    $<$<BOOL:${CGAL_FOUND}>:CGAL::CGAL>  # 条件链接CGAL
)

# Distortion Viewer GUI (新的失真可视化工具)
add_executable(DistortionViewer
    gui/src/distortion_viewer_main.cpp
    gui/src/imgui_distortion_viewer.cpp
)

target_include_directories(DistortionViewer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/core/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/geometry-central/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/glm
)

target_link_libraries(DistortionViewer
    stm-core
    geometry-central
    Eigen3::Eigen
    ${OPENGL_LIBRARIES}
    GLEW::GLEW
    glfw
    imgui::imgui
    ${CMAKE_THREAD_LIBS_INIT}
)

# Spot测试程序
add_executable(test_spot examples/test_spot.cpp)
target_link_libraries(test_spot
    stm-core
    Eigen3::Eigen
)

# 集成测试程序
add_executable(test_integration
    test_integration.cpp
)
target_link_libraries(test_integration
    stm-core
    cuts-core
    flatten-core
    core
    geometry-central
    Eigen3::Eigen
)

# 真实算法测试程序
add_executable(test_real_algorithms
    test_real_algorithms.cpp
    gui/src/real_algorithms.cpp
)
target_include_directories(test_real_algorithms PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/gui/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/geometry-central/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../deps/glm
)
target_link_libraries(test_real_algorithms
    geometry-central
    Eigen3::Eigen
    ${CMAKE_THREAD_LIBS_INIT}
)

# 添加示例程序子目录
add_subdirectory(examples)

# 可视化测试程序
add_executable(test_visualization examples/test_visualization.cpp)
target_link_libraries(test_visualization
    stm-core
    Eigen3::Eigen
)
set_target_properties(test_visualization PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)

# 添加测试程序子目录
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/tests/CMakeLists.txt)
    add_subdirectory(tests)
endif()

# 编译选项
if(MSVC)
    # 只为存在的目标设置编译选项
    if(TARGET ${PROJECT_NAME})
        target_compile_options(${PROJECT_NAME} PRIVATE /W4)
    endif()
    if(TARGET ${PROJECT_NAME}_imgui_gui)
        target_compile_options(${PROJECT_NAME}_imgui_gui PRIVATE /W4 /wd4201)  # 禁用C4201警告(GLM库的无名称结构/联合)
    endif()
    if(TARGET DistortionViewer)
        target_compile_options(DistortionViewer PRIVATE /W4 /wd4201)
    endif()
    if(TARGET test_spot)
        target_compile_options(test_spot PRIVATE /W4)
    endif()
else()
    if(TARGET ${PROJECT_NAME})
        target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -pedantic)
    endif()
    if(TARGET ${PROJECT_NAME}_imgui_gui)
        target_compile_options(${PROJECT_NAME}_imgui_gui PRIVATE -Wall -Wextra -pedantic)
    endif()
    if(TARGET DistortionViewer)
        target_compile_options(DistortionViewer PRIVATE -Wall -Wextra -pedantic)
    endif()
    if(TARGET test_spot)
        target_compile_options(test_spot PRIVATE -Wall -Wextra -pedantic)
    endif()
endif()

# 复制数据文件到构建目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../../data/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)

# 设置Visual Studio启动项目
set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME}_imgui_gui)