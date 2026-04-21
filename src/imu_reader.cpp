#include     <stdio.h>      /*标准输入输出的定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*UNIX 标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>  
#include     <fcntl.h>	    /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <sys/time.h>
#include  <string.h>
#include  <getopt.h>
#include "imu_reader.h"
#include "imu_data.h"
#include <setpriority.h>


ImuReader::ImuReader() : imu_running(true)
{
    fd = open(Device, O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY); 
    if (fd < 0)	{
        printf("Can't Open Serial Port!\n");	
    }
	
    printf("open serial port to decode msg!\n");

    //save to oldtio
    tcgetattr(fd, &oldtio);
    bzero(&newtio, sizeof(newtio));
    newtio.c_cflag = speed | CS8 | CLOCAL | CREAD;
    newtio.c_cflag &= ~CSTOPB;
    newtio.c_cflag &= ~PARENB;
    newtio.c_iflag = IGNPAR;  
    newtio.c_oflag = 0;
    tcflush(fd,TCIFLUSH);  
    tcsetattr(fd,TCSAFLUSH,&newtio);  
    tcgetattr(fd,&oldtio);
	
    memset(buffer,0,sizeof(buffer)); 


    // 启动线程
    _imu_read_thread = std::thread(&ImuReader::run, this);

    printf("_imu_read_thread 启动!\n");
    // 设置线程优先级
    int max_priority = sched_get_priority_max(SCHED_FIFO) - 20;
    set_thread_priority(_imu_read_thread, max_priority);
}

void ImuReader::set_thread_priority(std::thread& th, int priority) {
    sched_param sch_params;
    sch_params.sched_priority = priority;
    if (pthread_setschedparam(th.native_handle(), SCHED_FIFO, &sch_params) != 0) {
        perror("Failed to set thread priority");
    }
}

ImuReader::~ImuReader() {
    imu_running = false; // 停止运行标志
    if (_imu_read_thread.joinable()) {
        _imu_read_thread.join(); 
    }
    
    // close 需要传入文件描述符 fd
    if (fd >= 0) {
        ::close(fd); 
    }
}
void ImuReader::ImuDateRead() {
    // 1. 缓冲区安全检查
    int free_space = sizeof(g_recv_buf) - g_recv_buf_idx;
    if (free_space <= 0) {
        g_recv_buf_idx = 0; // 溢出清空
        free_space = sizeof(g_recv_buf);
    }

    nread = ::read(fd, buffer, (free_space < RX_BUF_LEN) ? free_space : RX_BUF_LEN);
    if (nread > 0) {
        memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);
        g_recv_buf_idx += nread;
    }

    // 注意：这里不要再写 unsigned short cnt... 而是直接操作逻辑变量
    int current_cnt = g_recv_buf_idx;
    int current_pos = 0;

    while (current_cnt >= YIS_OUTPUT_MIN_BYTES) {
        int ret = analysis_data(g_recv_buf + current_pos, current_cnt, &g_output_info);
        
        if (analysis_ok == ret) {
            // 解析成功
            output_data_header_t *header = (output_data_header_t *)(g_recv_buf + current_pos);
            int frame_total_len = header->len + YIS_OUTPUT_MIN_BYTES;
            
            // printf("pitch: %f, roll: %f, yaw: %f\n", 
            //        g_output_info.attitude.pitch, g_output_info.attitude.roll, g_output_info.attitude.yaw);
            
            current_pos += frame_total_len;
            current_cnt -= frame_total_len;
        } 
        else if (data_len_err == ret) {
            // 重点：如果当前剩下的数据确实还没到一个完整包的长度，才跳出等待
            // 如果 header->len 明显不合理，说明是伪帧头，应该当做 analysis_done 处理
            output_data_header_t *header = (output_data_header_t *)(g_recv_buf + current_pos);
            if (header->header1 == PROTOCOL_FIRST_BYTE && header->len > 200) { // 假设最大包长度不超过200
                current_pos++;
                current_cnt--;
                // printf("未查找到帧头");
            } else {
                // printf("长度错误");
                break; // 长度真的不够，等下次 read
            }
        }
        else if (crc_err == ret) {
            // CRC 错误，说明这包坏了，跳过帧头
            current_pos++;
            current_cnt--;
        }
        else { // analysis_done (未查找到帧头)
            current_pos++;
            current_cnt--;
        }
    }

    // 搬运剩余数据
    if (current_cnt > 0 && current_pos > 0) {
        memmove(g_recv_buf, g_recv_buf + current_pos, current_cnt);
    }
    g_recv_buf_idx = current_cnt;
}

void ImuReader::compute() {
    
    // 1. 读取原始数据
    float raw_pitch = g_output_info.attitude.pitch;
    float raw_roll  = g_output_info.attitude.roll;
    float raw_yaw   = g_output_info.attitude.yaw;

    // 2. 软件滤波 
    float f_pitch = filter_p.update(raw_pitch);
    float f_roll  = filter_r.update(raw_roll);

    // 3. 自动校准逻辑
    static float pitch_offset = 0.0f;
    static float roll_offset  = 0.0f;

    if (!is_calibrated) {
    // 累计样本
    if (calib_cnt < CALIB_TARGET) {
        pitch_sum += f_pitch;
        roll_sum  += f_roll;
        calib_cnt++;
        
        // 校准期间可以输出0，防止系统误动作
        this->pitch = 0;
        this->roll  = 0;
        return; 
    } else {
        // 计算平均偏移量
        pitch_offset = pitch_sum / (float)CALIB_TARGET;
        roll_offset  = roll_sum / (float)CALIB_TARGET;
        is_calibrated = true;
        std::cout << "IMU Calibration Success! P_off: " << pitch_offset << " R_off: " << roll_offset << std::endl;
    }
}

    // 4. 应用校准值
    this->pitch = f_pitch - pitch_offset;
    this->roll  = f_roll - roll_offset;
    this->yaw   = raw_yaw; 

    // 调试打印
    // static int print_cnt = 0;

    // if(print_cnt++ % 100 == 0) {
    // std::cout << "pitch: " << this->pitch << " roll: " << this->roll << std::endl;
    // }
}


void ImuReader::run() {
    while (imu_running) {
        // 降低 CPU 占用，且保证实时性
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        
        ImuDateRead();
        compute();
    }
}