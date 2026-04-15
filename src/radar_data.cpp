#include "radar_data.h"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <thread>
#include <chrono>
#include <sched.h>


// 静态成员定义
const uint8_t RadarReceiver::RADAE_HEADER[1] = {0xAA};
const uint8_t RadarReceiver::RADAE_TAIL[1] = {0x0A};

// 构造函数
RadarReceiver::RadarReceiver() 
    : fd(-1), recv_len(0){
    memset(recv_buf, 0, sizeof(recv_buf));
    if (!initialize()) 
    {
        std::cerr << "Failed to initialize USB RC receiver" << std::endl;
        running_ = false;  // 确保线程不会启动
        return;
    }

    // 创建RC接收线程
    _RC_thread = std::thread(&RadarReceiver::RC_thread_function, this);

    int max_priority = sched_get_priority_max(SCHED_FIFO);
    int min_priority = sched_get_priority_min(SCHED_FIFO);

}

// 析构函数
RadarReceiver::~RadarReceiver() {
    running_ = false;
    if (_RC_thread.joinable()) {
        _RC_thread.join();
    }
    close();
}

// 串口配置函数
int RadarReceiver::openSerial(const char* device) {
    int serial_fd = -1;
    

    serial_fd = open(Device, O_RDWR | O_NOCTTY | O_SYNC);
    
    if (serial_fd >= 0) {
        // 设备打开成功
        std::cout << "成功打开设备: " << Device << std::endl;
        device_path = Device;
    } 
    else 
    {
        // 设备打开失败
        std::cout << "尝试打开设备: " << Device << " 失败" << std::endl;
        std::cout << "无可用的雷达设备" << std::endl;
        return -1;
    }
    
    
    if (serial_fd < 0) {
        return -1; // 所有设备都打开失败
    }
    
    struct termios tty{};
    if (tcgetattr(serial_fd, &tty) != 0) {
        perror("tcgetattr");
        ::close(serial_fd);
        return -1;
    }

    cfsetospeed(&tty, B115200);
    cfsetispeed(&tty, B115200);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;         // disable break processing
    tty.c_lflag = 0;                // no signaling chars, no echo,
                                   // no canonical processing
    tty.c_oflag = 0;                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;            // read doesn't block
    tty.c_cc[VTIME] = 0;            // 立即返回，最小值（配合10ms发送频率）

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

    tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
                                    // enable reading
    tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_fd, TCSANOW, &tty) != 0) {
        perror("tcsetattr");
        ::close(serial_fd);
        return -1;
    }

    return serial_fd;
}

// 查找包头
int RadarReceiver::findPacketHeader(const uint8_t* buf, int len) {
    for (int i = 0; i <= len - 1; ++i) {
        if (memcmp(buf + i, RADAE_HEADER, 1) == 0) {
            return i;
        }
    }
    return -1;
}

// 解析数据包
bool RadarReceiver::parseRadar(const uint8_t* radar, RadarData& data) {
 
    data.x_pos = (radar[1] << 8) | radar[2];
    data.y_pos = (radar[3] << 8) | radar[4];
    data.z_pos = (radar[5] << 8) | radar[6];
    data.yaw_pos = (radar[7] << 8) | radar[8];


    std::cout << "x_pos: " << data.x_pos << std::endl;
    std::cout << "y_pos: " << data.y_pos << std::endl;
    std::cout << "z_pos: " << data.z_pos << std::endl;
    std::cout << "yaw_pos: " << data.yaw_pos << std::endl;
    return true;
}

// 初始化连接
bool RadarReceiver::initialize() {
    fd = openSerial(device_path);
    return fd >= 0;
}

// radar线程函数
void RadarReceiver::RC_thread_function() {
    while (running_) {
        auto t = std::chrono::high_resolution_clock::now();
        t += std::chrono::microseconds(1000);
        
        readRadarData();
        
       
        std::this_thread::sleep_until(t);
    }
}

// 读取RC数据
bool RadarReceiver::readRadarData() {
    if (fd < 0) return false;
 
    int n = read(fd, recv_buf + recv_len, sizeof(recv_buf) - recv_len);
    if (n > 0) {
        recv_len += n;
        
        // 查找包头
        while (recv_len >= RADAE_SIZE) {
            int idx = findPacketHeader(recv_buf, recv_len);
           
            if (idx >= 0 && recv_len - idx >= RADAE_SIZE) {
                // 检查包尾
                if (memcmp(recv_buf + idx + RADAE_SIZE - 1, RADAE_TAIL, 1) == 0) {
                    // 解析数据
                    parseRadar(recv_buf + idx, _RadarData);

                    // 移除已处理数据
                    int remain = recv_len - (idx + RADAE_SIZE);
                    memmove(recv_buf, recv_buf + idx + RADAE_SIZE, remain);
                    recv_len = remain;
                    return true;
                } else {
                    // 包尾错误，丢弃这个包头开始的部分数据，避免死循环
                    memmove(recv_buf, recv_buf + idx + 1, recv_len - idx - 1);
                    recv_len -= (idx + 1);
                    
                }
            } else if (idx < 0) {
                // 包头没找到，清理数据，避免缓冲区溢出
                if (recv_len > RADAE_SIZE) {
                    recv_len = 0;
                }
                break;
            } else {
                // 数据不完整，等待更多数据
                break;
            }
        }
    } else if (n < 0) {
        perror("read");
        return false;
    }

    return false;  // 没有完整数据包
}

// 关闭连接
void RadarReceiver::close() {
    running_ = false;
    if (fd >= 0) {
        ::close(fd);
        fd = -1;
        recv_len = 0;
    }
}