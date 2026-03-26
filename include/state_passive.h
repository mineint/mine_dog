#pragma once

class State_Passive : public FSM_State {
public:
    State_Passive(FSM_Data* data) : FSM_State(data) 
    {
        currentStateName_ = FSM_StateName::PASSIVE;
    }

    void onEnter() override;

    

    void runState() override;
    FSM_StateName checkTransition() override;

    void onExit() override;
};
