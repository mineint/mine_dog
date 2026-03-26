#include "dog_fsm.h"
#include "leg_controller.h"
#include "state_passive.h"
#include "leg_kinematics.h"



void State_Passive::onEnter(){
    // 设置所有腿为被动，力矩0

    _data->leg_controller->sendZeroTorques(_data->can_ptr.get());
    _data->start_high = -0.06;
    std::cout << "Entered PASSIVE" << std::endl;
}

void State_Passive::runState(){

    if (!_data) {
        std::cerr << "[Passive] _data is null!\n";
        return;
    }

    if (!_data->leg_controller) {
        std::cerr << "[Passive] leg_controller is null!\n";
        return;
    }

    if (!_data->can_ptr) {
        std::cerr << "[Passive] can_ptr is null!\n";
        return;
    }

    _data->leg_controller->sendZeroTorques(_data->can_ptr.get()); 

    



}

FSM_StateName State_Passive::checkTransition(){
    //std::cout <<  "State_Passive检查切换" << _data->command << std::endl;
    if (_data->command == "stand") 
    {
        return FSM_StateName::STAND;
    }
    if (_data->command == "trot") 
    {
        return FSM_StateName::TROT;
    }
    else
    {
        return FSM_StateName::PASSIVE;
    }
}

void State_Passive::onExit(){
    std::cout << "PASSIVE onExit" << std::endl;
}

