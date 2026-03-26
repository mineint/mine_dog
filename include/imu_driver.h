#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include "dog_imu.h"

class ImuDriver {
public:
    ImuDriver() : fd_(-1), buffer_idx_(0) {
        std::memset(&output_info_, 0, sizeof(output_info_));
        std::memset(recv_buf_, 0, sizeof(recv_buf_));
    }

    ~ImuDriver() { closeSerial(); }

    // 初始化：注意默认波特率 460800
    bool init(const std::string& port = "/dev/ttyUSB0", int baudrate = B460800) {
        fd_ = open(port.c_str(), O_RDWR | O_NONBLOCK | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) return false;

        struct termios newtio;
        std::memset(&newtio, 0, sizeof(newtio));
        newtio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
        newtio.c_cflag &= ~CSTOPB;
        newtio.c_cflag &= ~PARENB;
        newtio.c_iflag = IGNPAR;
        newtio.c_oflag = 0;
        
        tcflush(fd_, TCIFLUSH);
        tcsetattr(fd_, TCSAFLUSH, &newtio);
        return true;
    }

    // 核心更新逻辑：处理 0x59 0x53 协议流
    bool update() {
        unsigned char read_buf[512];
        // 1. 读取当前串口内所有可用数据
        int nread = read(fd_, read_buf, sizeof(read_buf));
        if (nread > 0) {
            // 将新读到的数据拼接到缓冲区末尾
            if (buffer_idx_ + nread < MAX_BUF_SIZE) {
                std::memcpy(recv_buf_ + buffer_idx_, read_buf, nread);
                buffer_idx_ += nread;
            } else {
                buffer_idx_ = 0; // 溢出重置
            }
        }

        // 2. 如果缓冲区还没达到一帧的最小长度(7字节)，继续等待
        if (buffer_idx_ < YIS_OUTPUT_MIN_BYTES) return false;

        bool found_valid_frame = false;
        int pos = 0;

        // 3. 滑动窗口解析
        // 只要缓冲区剩下的长度可能包含一帧，就持续解析
        while (pos <= buffer_idx_ - YIS_OUTPUT_MIN_BYTES) {
            
            // 调用官方提供的解析函数
            int ret = analysis_data(recv_buf_ + pos, buffer_idx_ - pos, &output_info_);

            if (ret == analysis_ok) {
                // 解析成功！根据协议计算这一帧的总长度
                // 帧头(2) + TID(2) + Len(1) + Payload(header->len) + Checksum(2)
                output_data_header_t* header = (output_data_header_t*)(recv_buf_ + pos);
                int frame_total_len = header->len + YIS_OUTPUT_MIN_BYTES;
                
                pos += frame_total_len;
                found_valid_frame = true;
                // 继续在剩下的缓冲区里找下一帧（防止数据堆积）
            } 
            else if (ret == analysis_done) {
                // 当前位置不是 0x59 0x53，往后移 1 字节继续找
                pos++;
            } 
            else if (ret == data_len_err) {
                // 剩下的数据不够一整帧了，跳出循环等待更多数据读取
                break;
            } 
            else {
                // 其他错误（如校验错误 crc_err），跳过当前字节继续找
                pos++;
            }
        }

        // 4. 整理缓冲区：将未处理/不完整的残余数据移到最前面
        if (pos > 0) {
            int remain = buffer_idx_ - pos;
            if (remain > 0) {
                std::memmove(recv_buf_, recv_buf_ + pos, remain);
                buffer_idx_ = remain;
            } else {
                buffer_idx_ = 0;
            }
        }

        return found_valid_frame;
    }

    const protocol_info_t& getImuData() const { return output_info_; }

    void closeSerial() {
        if (fd_ >= 0) { close(fd_); fd_ = -1; }
    }

private:
    int fd_;
    static const int MAX_BUF_SIZE = 2048; // 加大缓冲区
    unsigned char recv_buf_[MAX_BUF_SIZE];
    int buffer_idx_;
    protocol_info_t output_info_;
};