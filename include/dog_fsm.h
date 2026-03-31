#pragma once

#include <memory> 
#include "gait_scheduler.h"
#include "leg_controller.h" 
#include "leg_kinematics.h"
#include "KeyboardReader.h"
#include "leg_swing.h"
#include "dog_data.h"

 

enum class FSM_StateName {  
    PASSIVE,
    STAND,
    TROT,
    JUMP,
};

class FSM_State {
public:
    explicit FSM_State(FSM_Data* data) : _data(data) {}
    virtual ~FSM_State() = default;

    virtual void onEnter() = 0;
    virtual void runState() = 0;
    virtual FSM_StateName checkTransition() = 0;
    virtual void onExit() = 0;
    
    FSM_StateName currentStateName_ = FSM_StateName::PASSIVE;
    FSM_StateName nextStateName_ = FSM_StateName::PASSIVE;
    
protected:
    FSM_Data* _data;    
    
};

class FSM {
public:
    explicit FSM(std::shared_ptr<Tangair_usb2can> can = nullptr, std::shared_ptr<ImuReader> imu = nullptr);

    // 主更新函数
    void update(double dt);

    // 状态切换
    void transitionTo(FSM_StateName next);

    void update_motor(Tangair_usb2can* can_device);

    void update_imu(ImuReader* imu_ptr);

    // 数据访问
    FSM_Data* getData() { return _data.get(); }
    const FSM_Data* getData() const { return _data.get(); }

    KeyboardReader* readerKey;

private:
    std::unique_ptr<FSM_Data> _data;
    std::unique_ptr<FSM_State> current_state_;  
};

