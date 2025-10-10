/**
 * GLFW存根函数 - 解决版本兼容性问题
 * 为缺失的GLFW 3.4+函数提供空实现
 */

#include <GLFW/glfw3.h>

// 定义缺失的常量和结构
#ifndef GLFW_NO_ERROR
#define GLFW_NO_ERROR 0
#endif

#ifndef GLFW_FALSE
#define GLFW_FALSE 0
#endif

// 定义缺失的结构体
typedef struct GLFWgamepadstate {
    unsigned char buttons[15];
    float axes[6];
} GLFWgamepadstate;

// 为ImGui所需的GLFW 3.4+函数提供存根实现
extern "C" {
    // glfwGetError - GLFW 3.3+中引入
    int glfwGetError(const char** description) {
        if (description) *description = "GLFW error checking not available in this version";
        return GLFW_NO_ERROR;
    }

    // glfwGetGamepadState - GLFW 3.3+中引入
    int glfwGetGamepadState(int jid, GLFWgamepadstate* state) {
        // 简单返回失败，表示没有手柄
        (void)jid;
        (void)state;
        return GLFW_FALSE;
    }
}