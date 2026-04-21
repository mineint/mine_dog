#pragma once
#include "imu_data.h"
#include <thread>
#include <atomic>
#include <termios.h>

#define TRUE 		1
#define FALSE 		-1
#define RX_BUF_LEN	512

struct LowPassFilter {
    float alpha = 0.2f; 
    float state = 0.0f;
    float update(float val) {
        state = alpha * val + (1.0f - alpha) * state;
        return state;
    }
};
class ImuReader
{
private:
    int fd;
    int nread;
    char buffer[RX_BUF_LEN];
    const char *Device = "/dev/ttyUSB0";
    struct termios oldtio,newtio;


    unsigned short cnt = 0;
    int pos = 0;

    speed_t speed = B460800;

    unsigned char g_recv_buf[512] = {0};
    unsigned short g_recv_buf_idx = 0;

public:
    ImuReader();
    ~ImuReader();

    void ImuDateRead();
    void run();
    void compute();
    void set_thread_priority(std::thread& th, int priority);
    
    protocol_info_t g_output_info = {0};
    std::thread _imu_read_thread;
    std::atomic<bool> imu_running;

    float pitch;					
	float roll;
	float yaw;

    LowPassFilter filter_p;
    LowPassFilter filter_r;
    LowPassFilter filter_gx;
    LowPassFilter filter_gy;

    bool is_calibrated = false;
    int calib_cnt = 0;
    float pitch_sum = 0, roll_sum = 0;
    const int CALIB_TARGET = 200; // 取200个样本求平均
};


