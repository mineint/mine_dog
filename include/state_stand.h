#pragma once

class State_Stand : public FSM_State {
public:
    State_Stand(FSM_Data* data) : FSM_State(data)
    {
        currentStateName_ = FSM_StateName::STAND;
    }

    void onEnter() override;

    void runState() override;

    FSM_StateName checkTransition() override;

    void onExit() override;
    
    // 缓起动相关参数
    double progress;
    double smooth_step;
    double torques_progress;
    double torques_smooth_step;

    float half_L = 0.4; 
    float half_W = 0.1; 
    
    float z_pitch_comp;
    float z_roll_comp;
    float k_comp = 0.5; // 补偿强度
};

