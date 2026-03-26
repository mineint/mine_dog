#pragma once

#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <pthread.h>
#include <iostream>
#include "dog_fsm.h"

class FSM; 

class ControlFSM {
public:
    // 1. 构造函数：在这里接收并保存 FSM 指针
    ControlFSM(std::shared_ptr<FSM> fsm) : _fsm(fsm) {
        // 初始化频率
        thread_frequency_hz = 1000; 
        running_.store(false);
    }

    ~ControlFSM() {
        stopFSMThread();
    }

    // 启动线程
    void startFSMThread() {
        if (!running_.load()) {
            running_.store(true);
            fsm_thread = std::thread(&ControlFSM::FSMThreadFunction, this);
            
            // 设置线程优先级 (可选)
            struct sched_param param;
            param.sched_priority = 30;
            pthread_setschedparam(fsm_thread.native_handle(), SCHED_FIFO, &param);
        }
    }

    // 停止线程
    void stopFSMThread() {
        if (running_.load()) {
            running_.store(false);
            if (fsm_thread.joinable()) {
                fsm_thread.join();
            }
        }
    }

private:
    // 2. 线程函数实现
    void FSMThreadFunction() {
        // 计算 dt 和 周期
        const double dt = 1.0 / thread_frequency_hz; 
        const auto loop_duration = std::chrono::microseconds(1000000 / thread_frequency_hz);
        
        auto next_wake = std::chrono::high_resolution_clock::now();

        while (running_.load()) {
            next_wake += loop_duration;

            // --- 调用 FSM 更新  ---
            if (_fsm) { 
                _fsm->update(dt); 
            }

            // --- 执行精确休眠 ---
            std::this_thread::sleep_until(next_wake);
            
            // 超时检查
            if (std::chrono::high_resolution_clock::now() > next_wake) {
                next_wake = std::chrono::high_resolution_clock::now();
            }
        }

        // 安全退出：发送停机指令
        std::cout << "[ControlFSM] Thread safe exit." << std::endl;
    }

    // 3. 成员变量定义 (这些必须写在类里)
    std::shared_ptr<FSM> _fsm;          // 对应你报错的 fsm_ptr
    int thread_frequency_hz = 1000;    // 对应你报错的 thread_frequency_hz
    std::atomic<bool> running_{false};
    std::thread fsm_thread;
};

