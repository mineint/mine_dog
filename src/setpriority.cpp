#include <setpriority.h>

void set_thread_priority(std::thread &t, int priority)
{
    pthread_t native_handle = t.native_handle(); // 获取原生线程句柄

    // 设置线程调度策略为 FIFO
    sched_param sch_params;
    sch_params.sched_priority = priority;

    if (pthread_setschedparam(native_handle, SCHED_FIFO, &sch_params) != 0)
    {
        std::cerr << "Error setting thread priority!" << std::endl;
    }
    else
    {
        std::cout << "Thread priority set to " << priority << std::endl;
    }
}