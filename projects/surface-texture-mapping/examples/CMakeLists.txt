# 示例程序构建配置
cmake_minimum_required(VERSION 3.10)

# 包含父目录的头文件（新的模块化结构）
include_directories(../core/include)
include_directories(../gui/include)
include_directories(../../../deps)
include_directories(../../../deps/geometry-central/include)
include_directories(../../../deps/libigl/include)

# 查找依赖
find_package(Eigen3 REQUIRED)

# 基础示例
add_executable(example_basic example_basic.cpp)
target_link_libraries(example_basic
    stm-core  # 使用新的模块化核心库
    Eigen3::Eigen
)

# 完整流水线示例
add_executable(example_full_pipeline example_full_pipeline.cpp)
target_link_libraries(example_full_pipeline
    stm-core  # 使用新的模块化核心库
    Eigen3::Eigen
)

# 复制数据文件到构建目录
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/../../../data/
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/data/)