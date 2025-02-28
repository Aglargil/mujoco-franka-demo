#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <GLFW/glfw3.h>
#include "mujoco/mujoco.h"

// MuJoCo 数据结构
mjModel* m = NULL;                  // MuJoCo 模型
mjData* d = NULL;                   // MuJoCo 数据
mjvCamera cam;                      // 抽象相机
mjvOption opt;                      // 可视化选项
mjvScene scn;                       // 抽象场景
mjrContext con;                     // 渲染上下文

// 鼠标交互
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;

// 键盘回调函数
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods) {
    // 按下 ESC 退出
    if (act == GLFW_PRESS && key == GLFW_KEY_ESCAPE)
        glfwSetWindowShouldClose(window, GLFW_TRUE);
}

// 鼠标按钮回调函数
void mouse_button(GLFWwindow* window, int button, int act, int mods) {
    // 更新按钮状态
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    // 更新光标位置
    glfwGetCursorPos(window, &lastx, &lasty);
}

// 鼠标移动回调函数
void mouse_move(GLFWwindow* window, double xpos, double ypos) {
    // 计算鼠标位移
    if (!button_left && !button_middle && !button_right)
        return;

    // 计算位移
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // 根据按下的按钮确定交互类型
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    if (button_right)
        mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*dy, &scn, &cam);
    else if (button_left)
        mjv_moveCamera(m, mjMOUSE_ROTATE_V, dx/width, dy/height, &scn, &cam);
    else if (button_middle)
        mjv_moveCamera(m, mjMOUSE_MOVE_V, dx/width, -dy/height, &scn, &cam);
}

// 滚轮回调函数
void scroll(GLFWwindow* window, double xoffset, double yoffset) {
    // 缩放视图
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, 0.05*yoffset, &scn, &cam);
}

// 主函数
int main(int argc, const char** argv) {
    // 初始化 GLFW
    if (!glfwInit())
        mju_error("无法初始化 GLFW");

    // 创建窗口
    GLFWwindow* window = glfwCreateWindow(1200, 900, "转发运动学演示", NULL, NULL);
    if (!window) {
        glfwTerminate();
        mju_error("无法创建 GLFW 窗口");
    }

    // 初始化 MuJoCo 上下文
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // 注册回调函数
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // 激活 MuJoCo
    mj_activate("path/to/your/mjkey.txt");  // 需要替换为您的许可证路径

    // 加载模型
    char error[1000] = "无法加载模型";
    m = mj_loadXML("model/franka_emika_panda/panda_nohand.xml", NULL, error, 1000);
    if (!m)
        mju_error_s("加载错误: %s", error);

    // 创建数据实例
    d = mj_makeData(m);

    // 初始化可视化结构
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);

    // 创建场景和上下文
    mjv_makeScene(m, &scn, 2000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    // 设置相机参数，与 Python 版本相匹配
    cam.distance = 3.0;
    cam.azimuth = 45.0;
    cam.elevation = -20.0;

    // 设置模拟参数
    int fps = 200;
    int N = 1000;
    double step_duration = 1.0 / fps;
    struct timespec start_time, current_time;
    double elapsed, remaining;

    // 模拟循环
    for (int i = 0; i < N && !glfwWindowShouldClose(window); i++) {
        // 记录开始时间
        clock_gettime(CLOCK_REALTIME, &start_time);

        // 计算控制输入
        double c = i * (M_PI / 2) / N;
        
        // 设置关节控制值
        for (int j = 0; j < m->nu; j++) {
            d->ctrl[j] = c;
        }

        // 前进一步模拟
        mj_step(m, d);

        // 渲染场景
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);

        // 交换缓冲区和处理事件
        glfwSwapBuffers(window);
        glfwPollEvents();

        // 控制帧率
        clock_gettime(CLOCK_REALTIME, &current_time);
        elapsed = (current_time.tv_sec - start_time.tv_sec) + 
                  (current_time.tv_nsec - start_time.tv_nsec) / 1e9;
        remaining = step_duration - elapsed;
        
        if (remaining > 0) {
            struct timespec ts;
            ts.tv_sec = (time_t)remaining;
            ts.tv_nsec = (long)((remaining - ts.tv_sec) * 1e9);
            nanosleep(&ts, NULL);
        }
    }

    // 清理
    mjv_freeScene(&scn);
    mjr_freeContext(&con);
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();
    
    // 终止 GLFW
    glfwTerminate();
    
    return 0;
} 