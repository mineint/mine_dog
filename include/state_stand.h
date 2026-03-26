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
};

