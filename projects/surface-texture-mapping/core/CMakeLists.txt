# Surface Texture Mapping Core Library
cmake_minimum_required(VERSION 3.10)

# 收集所有源文件
file(GLOB_RECURSE STM_CORE_SRCS src/*.cpp)
file(GLOB_RECURSE STM_CORE_HEADERS include/*.h include/*.hpp)

# 已修复：eulerian_cut_integrator.cpp现在正确调用setState()初始化优化器
# 修复日期：2025-10-10
# 修复内容：添加normalClusterMSDF初始化和setState()调用（第167-181行）
# 参考文档：docs/Variational Surface Cutting调用失败的根本原因分析.md
# list(FILTER STM_CORE_SRCS EXCLUDE REGEX ".*eulerian_cut_integrator\\.cpp$")  # 已重新启用

# 创建核心库
add_library(
    stm-core
    ${STM_CORE_SRCS}
    ${STM_CORE_HEADERS}
)

# 设置包含目录
target_include_directories(stm-core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_SOURCE_DIR}/projects/cuts/core/include  # EulerianShapeOptimizer头文件
    ${CMAKE_SOURCE_DIR}/projects/flatten/core/include  # BFF头文件
    ${CMAKE_SOURCE_DIR}/core/include  # 核心几何库头文件
    ${CMAKE_SOURCE_DIR}/deps/boundary-first-flattening/include  # BFF库头文件
)

# 链接依赖库
target_link_libraries(stm-core PUBLIC
    core  # 依赖主项目的core库
    cuts-core  # 依赖变分切割算法
    flatten-core  # 依赖BFF算法
    bff  # 链接真实的BFF库
    geometry-central
    Clipper2
    Eigen3::Eigen
    SuiteSparse::CHOLMOD  # 添加CHOLMOD链接
)

# 如果有SuiteSparse支持
if(HAVE_SUITESPARSE)
    target_link_libraries(stm-core ${SUITESPARSE_LIBRARIES})
endif()

# 设置C++17标准
target_compile_features(stm-core PUBLIC cxx_std_17)

# 编译选项
if(MSVC)
    target_compile_options(stm-core PRIVATE /W4)
else()
    target_compile_options(stm-core PRIVATE -Wall -Wextra -pedantic)
endif()