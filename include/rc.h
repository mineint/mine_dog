#pragma once

#include <cstdint>
#include <thread>

// RC数据结构
struct RCData
{
    uint8_t CH1;
    uint8_t CH2;
    uint8_t CH3;
    uint8_t CH4;
    uint8_t S1;
    uint8_t S2;
    uint8_t S3;
    uint8_t S4;
};

// USB RC接收器类
class USBRCReceiver
{
private:
    // 包定义
    static const uint8_t PACKET_HEADER[2];
    static const uint8_t PACKET_TAIL[2];
    static const int PACKET_SIZE = 15;
    static const int DATA_LEN = 10;
    static const int BUFFER_SIZE = 15;

    int fd;                        // 串口文件描述符
    uint8_t recv_buf[BUFFER_SIZE]; // 接收缓冲区
    int recv_len;                  // 当前缓冲区数据长度
    const char *device_path;       // 设备路径

    // 串口配置函数
    int openSerial(const char *device);

    // 查找包头
    int findPacketHeader(const uint8_t *buf, int len);

    // 解析数据包
    bool parsePacket(const uint8_t *packet, RCData &data);

    // RC线程函数
    void RC_thread_function();

    const char *Device = "/dev/ttyACM2";

public:
    // 构造函数
    explicit USBRCReceiver();

    // 析构函数
    ~USBRCReceiver();

    RCData _RCData{};

    std::thread _RC_thread;
    bool running_ = true;
    int delay_us = 10000; // 10ms延时，配合STM32发送频率

    // 初始化连接
    bool initialize();

    // 读取RC数据
    bool readRCData();

    // 关闭连接
    void close();

    // 检查连接状态
    bool isConnected() const { return fd >= 0; }
};
