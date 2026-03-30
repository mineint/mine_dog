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



ImuReader::ImuReader() : imu_running(true)
{
    dev = "/dev/ttyUSB0";	
    fd = open(dev, O_RDWR | O_NONBLOCK| O_NOCTTY | O_NDELAY); 
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
    int max_priority = sched_get_priority_max(SCHED_FIFO);
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
void ImuReader::ImuDateRead(){
    nread = read(fd, buffer, RX_BUF_LEN);
	if(nread > 0)
	{
	    //printf("nread = %d\n", nread);
	    memcpy(g_recv_buf + g_recv_buf_idx, buffer, nread);             
	    g_recv_buf_idx += nread;
	}

        cnt = g_recv_buf_idx;
        pos = 0;
        
        while(cnt > (unsigned int)0)
        {
            int ret = analysis_data(g_recv_buf + pos, cnt, &g_output_info);
            if(analysis_done == ret)	/*未查找到帧头*/
            {
                pos++;
                cnt--;
            }
            else if(data_len_err == ret)
            {
                break;
            }
            else if(crc_err == ret || analysis_ok == ret)	 /*删除已解析完的完整一帧*/
            {
                output_data_header_t *header = (output_data_header_t *)(g_recv_buf + pos);
                unsigned int frame_len = header->len + YIS_OUTPUT_MIN_BYTES;
                cnt -= frame_len;
                pos += frame_len;
                //memcpy(g_recv_buf, g_recv_buf + pos, cnt);

                if(analysis_ok == ret)
                {
                    printf("pitch: %f, roll: %f, yaw: %f\n", 
			  g_output_info.attitude.pitch, g_output_info.attitude.roll, g_output_info.attitude.yaw);
                }
	    }
	}

        memcpy(g_recv_buf, g_recv_buf + pos, cnt);
        g_recv_buf_idx = cnt;
	tcflush(fd,TCIFLUSH);
}

void ImuReader::run() {
    while (imu_running) {
        // 降低 CPU 占用，且保证实时性
        std::this_thread::sleep_for(std::chrono::microseconds(1000));
        ImuDateRead();
    }
}