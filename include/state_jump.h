#pragma once

class State_Jump : public FSM_State {
public:
    State_Stand(FSM_Data* data) : FSM_State(data)
    {
        currentStateName_ = FSM_StateName::JUMP;
    }

    void onEnter() override;

    void runState() override;

    FSM_StateName checkTransition() override;

    void onExit() override;
};