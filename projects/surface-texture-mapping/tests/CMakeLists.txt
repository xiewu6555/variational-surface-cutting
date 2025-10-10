# Surface Texture Mapping - 测试系统配置
# 版本: 1.0
# 日期: 2025-09-30

cmake_minimum_required(VERSION 3.10)

# ============================================================================
# 查找Google Test (使用CONFIG模式避免重复定义)
# ============================================================================
if(NOT TARGET GTest::gtest)
    find_package(GTest CONFIG REQUIRED)
endif()

# ============================================================================
# 通用测试配置
# ============================================================================

# 通用链接库列表
set(COMMON_TEST_LIBS
    stm-core
    geometry-central
    Eigen3::Eigen
    Clipper2
    GTest::gtest_main
    GTest::gtest
)

# 设置通用编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 添加包含目录
include_directories(${CMAKE_SOURCE_DIR}/projects/surface-texture-mapping/core/include)

# ============================================================================
# 辅助宏：添加测试可执行文件
# ============================================================================
macro(add_stm_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE} test_main.cpp)
    target_link_libraries(${TEST_NAME} PRIVATE ${COMMON_TEST_LIBS})
    set_target_properties(${TEST_NAME} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
    )
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})
endmacro()

# ============================================================================
# 单元测试 (Unit Tests)
# ============================================================================

# 新增核心模块测试 (最高优先级)
add_stm_test(test_distortion_analyzer test_distortion_analyzer.cpp)
add_stm_test(test_barycentric_mapper test_barycentric_mapper.cpp)
add_stm_test(test_pattern_back_mapper test_pattern_back_mapper.cpp)

# 基础模块测试 (待实现)
# add_stm_test(test_mesh_processor test_mesh_processor.cpp)
# add_stm_test(test_variational_cutter test_variational_cutter.cpp)
# add_stm_test(test_texture_mapper test_texture_mapper.cpp)
# add_stm_test(test_surface_filler test_surface_filler.cpp)

# ============================================================================
# 集成测试 (Integration Tests)
# ============================================================================

add_stm_test(test_real_space test_real_space.cpp)

# 三大核心模块集成测试 (使用真实模型)
add_stm_test(test_integration_core_modules test_integration_core_modules.cpp)

# 待实现的集成测试
# add_stm_test(test_uv_pipeline test_uv_pipeline.cpp)
# add_stm_test(test_cross_seam test_cross_seam.cpp)

# ============================================================================
# 端到端测试 (End-to-End Tests)
# ============================================================================

add_stm_test(test_e2e_cylinder test_e2e_cylinder.cpp)

# 待实现的端到端测试
# add_stm_test(test_e2e_spot test_e2e_spot.cpp)
# add_stm_test(test_quality_control test_quality_control.cpp)

# ============================================================================
# 原有测试 (保持兼容)
# ============================================================================

# 简单测试
add_executable(test_simple test_simple.cpp)
target_link_libraries(test_simple
    PRIVATE
        stm-core
        Eigen3::Eigen
)
set_target_properties(test_simple PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
)

# BFF算法测试
add_executable(test_bff test_bff.cpp)
target_link_libraries(test_bff
    PRIVATE
        stm-core
        core
        cuts-core
        geometry-central
        Eigen3::Eigen
)
set_target_properties(test_bff PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
)

# 完整流水线测试
add_executable(test_complete_pipeline test_complete_pipeline.cpp)
target_link_libraries(test_complete_pipeline
    PRIVATE
        stm-core
        geometry-central
        Clipper2
        Eigen3::Eigen
)
set_target_properties(test_complete_pipeline PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin/tests"
)

# 添加原有测试到CTest
add_test(NAME BFF_Algorithm_Test COMMAND test_bff)
add_test(NAME Complete_Pipeline_Test COMMAND test_complete_pipeline)

# ============================================================================
# 测试套件定义
# ============================================================================

# 定义测试标签，方便分组运行
set_tests_properties(
    test_distortion_analyzer
    test_barycentric_mapper
    test_pattern_back_mapper
    PROPERTIES LABELS "unit;core"
)

set_tests_properties(
    test_real_space
    PROPERTIES LABELS "integration"
)

set_tests_properties(
    test_e2e_cylinder
    PROPERTIES LABELS "e2e;slow"
)

# ============================================================================
# 自定义测试目标
# ============================================================================

# 快速测试：只运行单元测试
add_custom_target(test_quick
    COMMAND ${CMAKE_CTEST_COMMAND} -L unit --output-on-failure
    COMMENT "Running quick unit tests..."
)

# 完整测试：运行所有测试
add_custom_target(test_all
    COMMAND ${CMAKE_CTEST_COMMAND} --output-on-failure
    COMMENT "Running all tests..."
)

# 核心模块测试
add_custom_target(test_core
    COMMAND ${CMAKE_CTEST_COMMAND} -L core --output-on-failure
    COMMENT "Running core module tests..."
)

# 集成测试
add_custom_target(test_integration_suite
    COMMAND ${CMAKE_CTEST_COMMAND} -L integration --output-on-failure
    COMMENT "Running integration tests..."
)

# 端到端测试
add_custom_target(test_e2e
    COMMAND ${CMAKE_CTEST_COMMAND} -L e2e --output-on-failure
    COMMENT "Running end-to-end tests..."
)

# ============================================================================
# 测试数据目录
# ============================================================================

# 如果有测试数据，可以在这里复制到构建目录
# file(COPY ${CMAKE_SOURCE_DIR}/data/test_meshes
#      DESTINATION ${CMAKE_BINARY_DIR}/bin/tests/data)

# ============================================================================
# 代码覆盖率 (可选，需要gcov/lcov)
# ============================================================================

if(CMAKE_BUILD_TYPE STREQUAL "Debug" AND CMAKE_COMPILER_IS_GNUCXX)
    option(ENABLE_COVERAGE "Enable code coverage" OFF)
    if(ENABLE_COVERAGE)
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
        set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")

        add_custom_target(coverage
            COMMAND ${CMAKE_CTEST_COMMAND}
            COMMAND lcov --capture --directory . --output-file coverage.info
            COMMAND lcov --remove coverage.info '/usr/*' --output-file coverage.info
            COMMAND lcov --list coverage.info
            COMMAND genhtml coverage.info --output-directory coverage_html
            COMMENT "Generating code coverage report..."
        )
    endif()
endif()

# ============================================================================
# 测试报告生成
# ============================================================================

# 生成JUnit XML格式的测试报告 (用于CI集成)
add_custom_target(test_report
    COMMAND ${CMAKE_CTEST_COMMAND} --output-junit test_results.xml
    COMMENT "Generating test report..."
)

# ============================================================================
# 打印测试配置信息
# ============================================================================

message(STATUS "========================================")
message(STATUS "Surface Texture Mapping - Test Configuration")
message(STATUS "========================================")
message(STATUS "Google Test: ${GTEST_LIBRARIES}")
message(STATUS "Test output: ${CMAKE_BINARY_DIR}/bin/tests")
message(STATUS "========================================")
message(STATUS "Available test targets:")
message(STATUS "  make test          - Run CTest")
message(STATUS "  make test_quick    - Run unit tests only")
message(STATUS "  make test_all      - Run all tests")
message(STATUS "  make test_core     - Run core module tests")
message(STATUS "  make test_integration - Run integration tests")
message(STATUS "  make test_e2e      - Run end-to-end tests")
message(STATUS "========================================")