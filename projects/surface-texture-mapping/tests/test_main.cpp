/**
 * Google Test 主入口
 *
 * Surface Texture Mapping 项目测试套件
 * 统一的测试入口，支持命令行参数控制
 *
 * 使用方法:
 *   # 运行所有测试
 *   ./test_main
 *
 *   # 运行特定测试套件
 *   ./test_main --gtest_filter=DistortionAnalyzerTest.*
 *
 *   # 详细输出
 *   ./test_main --gtest_verbose
 *
 *   # 生成XML报告
 *   ./test_main --gtest_output=xml:test_results.xml
 *
 * 版本: 1.0
 * 日期: 2025-09-30
 */

#include <gtest/gtest.h>
#include <iostream>
#include <string>

/**
 * 自定义测试监听器
 * 用于输出额外的测试信息
 */
class CustomTestListener : public ::testing::EmptyTestEventListener {
public:
    void OnTestProgramStart(const ::testing::UnitTest& unit_test) override {
        std::cout << "\n";
        std::cout << "========================================\n";
        std::cout << "Surface Texture Mapping - Test Suite\n";
        std::cout << "========================================\n";
        std::cout << "Total test cases: " << unit_test.test_to_run_count() << "\n";
        std::cout << "========================================\n\n";
    }

    void OnTestIterationStart(const ::testing::UnitTest& unit_test, int iteration) override {
        if (iteration > 0) {
            std::cout << "\nStarting test iteration " << (iteration + 1) << "\n";
        }
    }

    void OnTestStart(const ::testing::TestInfo& test_info) override {
        std::cout << "[ RUN      ] " << test_info.test_suite_name()
                  << "." << test_info.name() << "\n";
    }

    void OnTestEnd(const ::testing::TestInfo& test_info) override {
        if (test_info.result()->Passed()) {
            std::cout << "[       OK ] " << test_info.test_suite_name()
                      << "." << test_info.name()
                      << " (" << test_info.result()->elapsed_time() << " ms)\n";
        } else {
            std::cout << "[  FAILED  ] " << test_info.test_suite_name()
                      << "." << test_info.name()
                      << " (" << test_info.result()->elapsed_time() << " ms)\n";
        }
    }

    void OnTestProgramEnd(const ::testing::UnitTest& unit_test) override {
        std::cout << "\n========================================\n";
        std::cout << "Test Summary\n";
        std::cout << "========================================\n";
        std::cout << "Total tests:  " << unit_test.test_to_run_count() << "\n";
        std::cout << "Passed:       " << unit_test.successful_test_count() << "\n";
        std::cout << "Failed:       " << unit_test.failed_test_count() << "\n";
        std::cout << "Disabled:     " << unit_test.disabled_test_count() << "\n";
        std::cout << "Elapsed time: " << unit_test.elapsed_time() << " ms\n";
        std::cout << "========================================\n";

        if (unit_test.Passed()) {
            std::cout << "\nAll tests PASSED!\n";
        } else {
            std::cout << "\nSome tests FAILED!\n";
        }
    }
};

/**
 * 测试环境设置
 */
class TestEnvironment : public ::testing::Environment {
public:
    void SetUp() override {
        // 全局测试环境初始化
        std::cout << "Initializing test environment...\n";

        // 可以在这里设置:
        // - 日志系统
        // - 临时文件目录
        // - 测试数据路径
        // - 等等

        std::cout << "Test environment initialized.\n\n";
    }

    void TearDown() override {
        // 全局测试环境清理
        std::cout << "\nCleaning up test environment...\n";
    }
};

/**
 * 主函数
 */
int main(int argc, char** argv) {
    // 初始化 Google Test
    ::testing::InitGoogleTest(&argc, argv);

    // 添加自定义监听器
    ::testing::TestEventListeners& listeners =
        ::testing::UnitTest::GetInstance()->listeners();

    // 移除默认的打印监听器 (可选)
    // delete listeners.Release(listeners.default_result_printer());

    // 添加自定义监听器
    listeners.Append(new CustomTestListener);

    // 添加全局测试环境
    ::testing::AddGlobalTestEnvironment(new TestEnvironment);

    // 打印版本信息
    std::cout << "\n";
    std::cout << "Project:  Surface Texture Mapping\n";
    std::cout << "Version:  1.0\n";
    std::cout << "Date:     2025-09-30\n";
    std::cout << "\n";

    // 打印命令行参数 (用于调试)
    if (argc > 1) {
        std::cout << "Command line arguments:\n";
        for (int i = 1; i < argc; ++i) {
            std::cout << "  " << argv[i] << "\n";
        }
        std::cout << "\n";
    }

    // 运行所有测试
    int result = RUN_ALL_TESTS();

    // 返回结果 (0 = 所有测试通过, 非0 = 有测试失败)
    return result;
}