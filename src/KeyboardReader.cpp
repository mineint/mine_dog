#include "KeyboardReader.h"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>

KeyboardReader::KeyboardReader() : running(true) {
    setupTerminal();
}

KeyboardReader::~KeyboardReader() {
    restoreTerminal();
}

void KeyboardReader::setupTerminal() {
    // 获取当前终端设置
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    
    // 设置为非规范模式，不回显
    new_tio.c_lflag &= ~(ICANON | ECHO);
    new_tio.c_cc[VMIN] = 0;  // 非阻塞读取
    new_tio.c_cc[VTIME] = 0;
    
    // 应用新设置
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);
    
    // 设置stdin为非阻塞
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);
}

void KeyboardReader::restoreTerminal() {
    // 恢复原始终端设置
    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);
}

char KeyboardReader::readKey() {
    char ch;
    int n = read(STDIN_FILENO, &ch, 1);
    if (n > 0) {
        return ch;
    }
    return 0;  // 没有按键
}

void KeyboardReader::processKey(char key) {
//     switch(key) {
//         case 'w':
//         case 'W':
//             std::cout << "前进" << std::endl;
//             break;
//         case 's':
//         case 'S':
//             std::cout << "后退" << std::endl;
//             break;
//         case 'a':
//         case 'A':
//             std::cout << "左移" << std::endl;
//             break;
//         case 'd':
//         case 'D':
//             std::cout << "右移" << std::endl;
//             break;
//         case 'q':
//         case 'Q':
//             std::cout << "左转" << std::endl;
//             break;
//         case 'e':
//         case 'E':
//             std::cout << "右转" << std::endl;
//             break;
//         case ' ':
//             std::cout << "停止" << std::endl;
//             break;
//         case '1':
//             std::cout << "步态: 慢走" << std::endl;
//             break;
//         case '2':
//             std::cout << "步态: 小跑" << std::endl;
//             break;
//         case '3':
//             std::cout << "步态: 跳跃" << std::endl;
//             break;
//         case 27: // ESC键
//             std::cout << "退出程序" << std::endl;
//             running = false;
//             break;
//         case '\n':
//         case '\r':
//             break;  // 忽略回车
//         default:
//             if (key != 0) {
//                 std::cout << "未知按键: " << (int)key << std::endl;
//             }
//             break;
//     }
}

void KeyboardReader::run() {
    // std::cout << "四足机器人键盘控制启动" << std::endl;
    // std::cout << "控制说明:" << std::endl;
    // std::cout << "W/S: 前进/后退" << std::endl;
    // std::cout << "A/D: 左移/右移" << std::endl;
    // std::cout << "Q/E: 左转/右转" << std::endl;
    // std::cout << "1/2/3: 步态切换" << std::endl;
    // std::cout << "空格: 停止" << std::endl;
    // std::cout << "ESC: 退出" << std::endl;
    // std::cout << "------------------------" << std::endl;
    
    while (running) {
        char key = readKey();
        if (key != 0) {
            processKey(key);
        }
        
        // 避免CPU占用过高
        usleep(10000); // 10ms延时
    }
}

void KeyboardReader::stop() {
    running = false;
}

bool KeyboardReader::isRunning() const {
    return running;
}