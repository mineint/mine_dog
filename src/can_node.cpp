// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0
#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <csignal>
#include <vector>
#include <memory>
#include <pthread.h>
#include "Tangair_usb2can.h"
#include "gait_scheduler.h"
#include "leg_controller.h" 
#include "leg_kinematics.h"
#include "leg_swing.h"
#include "dog_fsm.h"
#include "state_passive.h"
#include "state_stand.h"
#include "imu_driver.h"
#include "rc.h"

std::shared_ptr<USBRCReceiver> USBRCReceiver_ptr;

// 全局标志位
std::atomic<bool> should_exit{false};

class ControlFSM {
public:
    ControlFSM(std::shared_ptr<FSM> fsm, std::shared_ptr<Tangair_usb2can> can) 
        : _fsm(fsm), _can_ptr(can) {}
    //ControlFSM(std::shared_ptr<Tangair_usb2can> can) :  _can_ptr(can) {}
    ~ControlFSM() { stopFSMThread(); }

    void startFSMThread() {
        if (!running_.load()) {
            running_.store(true);
            fsm_thread = std::thread(&ControlFSM::FSMThreadFunction, this);
            
            // 1. 设置 CPU 亲和性 (绑定到核心 3)
            cpu_set_t cpuset;
            CPU_ZERO(&cpuset);
            CPU_SET(3, &cpuset);
            pthread_setaffinity_np(fsm_thread.native_handle(), sizeof(cpu_set_t), &cpuset);

            // 2. 设置线程优先级 (SCHED_FIFO)
            int max_priority = sched_get_priority_max(SCHED_FIFO);
            struct sched_param param;
            param.sched_priority = max_priority - 10; // 略低于最大值，保证系统不锁死
            pthread_setschedparam(fsm_thread.native_handle(), SCHED_FIFO, &param);

            std::cout << "[ControlFSM] Thread started on Core 3 with RT priority.\n";
        }
    }

    void stopFSMThread() {
        if (running_.load()) {
            running_.store(false);
            if (fsm_thread.joinable()) fsm_thread.join();
        }
    }

private:
    void FSMThreadFunction() {
        // 目标周期 1ms (1000Hz)
        const auto target_duration = std::chrono::microseconds(1000);
        const double dt = 1.0 / thread_frequency_hz; 
        while (running_.load()) {
            auto start_time = std::chrono::high_resolution_clock::now();

            _fsm->update(dt);     // dt = 0.001 主要用于步态，说明更新时长

            auto end_time = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);

            // 动态休眠：只休眠剩下的时间
            if (elapsed < target_duration) {
                std::this_thread::sleep_for(target_duration - elapsed);
            } else {
                // 如果运行时间超过了 1ms，不休眠，直接进入下一轮
                // 这种方式能最大程度减少卡顿
            }
        }

        // --- 安全退出：线程结束前彻底关闭电机 ---
        std::cout << "[ControlFSM] Thread exiting, disabling motors...\n";
        for (int i = 0; i < 10; i++) {
            if (_can_ptr) _can_ptr->DISABLE_ALL_MOTOR(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    std::atomic<bool> running_{false};
    std::thread fsm_thread;
    std::shared_ptr<Tangair_usb2can> _can_ptr;
    std::shared_ptr<FSM> _fsm; 
    int thread_frequency_hz = 1000;
};

// --- 信号处理 ---
void signal_handler(int signum) {
    std::cout << "\n[Main] Signal (" << signum << ") received. Exiting...\n";
    should_exit.store(true);
}

// --- 终端保护 ---
struct termios original_termios;
void restore_terminal() {
    tcsetattr(STDIN_FILENO, TCSANOW, &original_termios);
    int ret = system("stty sane");
    (void)ret; 
}

int main() {
    // 1. 系统准备
    tcgetattr(STDIN_FILENO, &original_termios);
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    // 2. 初始化硬件 (按你的顺序)
    auto CAN_ptr = std::make_shared<Tangair_usb2can>();

    // 3. 初始化 
    auto fsm_ptr = std::make_shared<FSM>(CAN_ptr); 
    auto ConFSM_ptr = std::make_shared<ControlFSM>(fsm_ptr, CAN_ptr);
    //USBRCReceiver_ptr = std::make_shared<USBRCReceiver>();
    //auto ConFSM_ptr = std::make_shared<ControlFSM>(CAN_ptr);
    // 初始化RC接收器

    /* if (!USBRCReceiver_ptr->initialize()) {
        std::cerr << "[WARNING] Failed to initialize USBRCReceiver, continuing without RC..." << std::endl;
    } else {
        std::cout << "[INFO] USBRCReceiver initialized successfully." << std::endl;
    } */

    // 4. 启动控制线程
    ConFSM_ptr->startFSMThread();

    // 5. 主循环 (低频监控)
    std::cout << "[Main] System running. Press Ctrl+C to stop.\n";
    while (!should_exit.load()) {
         /* std::cout << "RC Data - CH0: " << USBRCReceiver_ptr->_RCData.Left_X
                      << ", CH1: " << USBRCReceiver_ptr->_RCData.Left_Y
                      << ", CH2: " << USBRCReceiver_ptr->_RCData.Right_X
                      << ", CH3: " << USBRCReceiver_ptr->_RCData.Right_Y
                      << ", S1: " << (int)USBRCReceiver_ptr->_RCData.S1 
                      << ", S2: " << (int)USBRCReceiver_ptr->_RCData.S2 
                      << ", A: " << (int)USBRCReceiver_ptr->_RCData.A
                      << ", B: " << (int)USBRCReceiver_ptr->_RCData.B
                      << "\n";  */
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    // 6. 优雅退出流程
    std::cout << "[Main] Starting graceful shutdown...\n";
    
    // 第一步：停止控制线程（它会负责发送 DISABLE 指令）
    ConFSM_ptr->stopFSMThread();

    // 第二步：主线程再次确认关闭（双保险）
    if (CAN_ptr) {
        for (int i = 0; i < 5; i++) {
            CAN_ptr->DISABLE_ALL_MOTOR(100);
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

    // 第三步：恢复环境
    restore_terminal();
    std::cout << "[Main] Shutdown complete.\n";

    return 0;
}
// # Copyright (c) 2023-2025 TANGAIR 
// # SPDX-License-Identifier: Apache-2.0

