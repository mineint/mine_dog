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


void KeyboardReader::stop() {
    running = false;
}

bool KeyboardReader::isRunning() const {
    return running;
}