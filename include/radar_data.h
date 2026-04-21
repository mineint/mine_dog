#pragma once

#include <cstdint>
#include <thread>
#include <termios.h>

// RC数据结构
struct RadarData
{
    int16_t x_pos;
    int16_t y_pos;
    int16_t z_pos;
    int16_t yaw_pos;
};

// USB RC接收器类
class RadarReceiver
{
private:
    // 包定义
    static const uint8_t RADAE_HEADER[1];
    static const uint8_t RADAE_TAIL[1];
    static const int RADAE_SIZE = 10;
    static const int DATA_LEN = 8;
    static const int BUFFER_SIZE = 10;

    int fd;                        // 串口文件描述符
    uint8_t recv_buf[BUFFER_SIZE]; // 接收缓冲区
    int recv_len;                  // 当前缓冲区数据长度
    const char *device_path;       // 设备路径

    struct termios tty{};

    // 串口配置函数
    int openSerial(const char *device);

    // 查找包头
    int findPacketHeader(const uint8_t *buf, int len);

    // 解析数据包
    bool parseRadar(const uint8_t* radar, RadarData& data);

    // RC线程函数
    void RC_thread_function();

    const char *Device = "/dev/ttyUSB1";

public:
    // 构造函数
    explicit RadarReceiver();

    // 析构函数
    ~RadarReceiver();

    RadarData _RadarData{};

    std::thread _RC_thread;
    bool running_ = true;
    int delay_us = 10000; // 10ms延时，配合STM32发送频率

    // 初始化连接
    bool initialize();

    // 读取雷达数据
    bool readRadarData();

    // 关闭连接
    void close();

    // 检查连接状态
    bool isConnected() const { return fd >= 0; }
};
