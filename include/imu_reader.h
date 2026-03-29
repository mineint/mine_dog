#pragma once
#include "imu_data.h"

#define TRUE 		1
#define FALSE 		-1
#define RX_BUF_LEN	512



class imureader
{
private:
    int fd;
    int nread;
    char buffer[RX_BUF_LEN];
    const char* dev;
    struct termios oldtio,newtio;

    unsigned short cnt = 0;
    int pos = 0;

    speed_t speed = B460800;

    unsigned char g_recv_buf[512] = {0};
    unsigned short g_recv_buf_idx = 0;
    protocol_info_t g_output_info = {0};

public:
    imureader();
    void imudateread();
};


