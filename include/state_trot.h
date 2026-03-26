#pragma once

class State_Trot : public FSM_State {
public:
    State_Trot(FSM_Data* data) : FSM_State(data)
    {
        currentStateName_ = FSM_StateName::TROT;
    }

    void onEnter() override;

    void runState() override;

    FSM_StateName checkTransition() override;

    void onExit() override;
};

