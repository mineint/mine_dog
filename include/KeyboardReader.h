#pragma once

#include <termios.h>

class KeyboardReader {
private:
    struct termios old_tio, new_tio;
    bool running;
    

public:
    KeyboardReader();
    ~KeyboardReader();
    
    void setupTerminal();
    void restoreTerminal();
    char readKey();
    void stop();
    bool isRunning() const;
};
