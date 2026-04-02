#pragma once

class State_Jump : public FSM_State {
public:
    State_Jump(FSM_Data* data) : FSM_State(data)
    {
        currentStateName_ = FSM_StateName::JUMP;
    }

    void onEnter() override;

    void runState() override;

    FSM_StateName checkTransition() override;

    void onExit() override;

private:   
    // 记得清零time
    const double jump_time = 2.5;    // 跳跃总时长
    const double crouch_time = 1.0;   // 下蹲/起立时长
    const double flight_time_1 = 0.15;   // 阶段一飞行时长
    const double flight_time_2 = 0.15;   // 阶段二飞行时长
    const double flight_time = 0.4;  // 飞行时长
    const double crouch_high = -0.17;   // 下蹲高度
    const double rise_high = -0.28;   // 起立高度
    const double flight_high = -0.23;   // 飞行高度
    const double jump_back = -0.16;   // 跳跃时的x方向

    double last_high;      // 转化前的高度
    double crouch_progress = 0.0;

    Eigen::Vector3d flight_pos = {jump_back, 0.096, flight_high};
    

    Eigen::Vector3d jump_pDes;
};