#include "dog_fsm.h"
#include "leg_controller.h"
#include "state_passive.h"
#include "leg_kinematics.h"



void State_Passive::onEnter(){
    // 设置所有腿为被动，力矩0
    for (int i = 0; i < 5; i++) {
    _data->leg_controller->sendZeroTorques(_data->can_ptr.get());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    _data->timer = 0;   
    _data->stand_time = 2.0;
    _data->start_high = -0.06;
    _data->slow_torques_swith = true;
    std::cout << "Entered PASSIVE" << std::endl;
}

void State_Passive::runState(){

    // if (!_data) {
    //     std::cerr << "[Passive] _data is null!\n";
    //     return;
    // }

    // if (!_data->leg_controller) {
    //     std::cerr << "[Passive] leg_controller is null!\n";
    //     return;
    // }

    // if (!_data->can_ptr) {
    //     std::cerr << "[Passive] can_ptr is null!\n";
    //     return;
    // }
    _data->tx_count++;
    // Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(_data->leg_date[1].q);

     if (_data->tx_count % 1000 == 0)
            { 
            
            // std::cout << "foot_pos:" << foot_pos << std::endl;
            // std::cout << "tx_count:" << _data->tx_count << std::endl;

    // _data->leg_controller->sendZeroTorques(_data->can_ptr.get());
            } 
 

    // Eigen::Vector3d p_des = {0, -0.0003, -0.20};
    // Eigen::Vector3d q_des = _data->swing_controller->inverseKinematics(p_des);
    // Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(q_des);

    // if (_data->tx_count % 100 == 0)
    //         { 
            
    //         std::cout << "foot_pos:" << foot_pos << std::endl;
    //         }

}

// passive仅可以切换为stand
FSM_StateName State_Passive::checkTransition(){
    // std::cout <<  "State_Passive检查切换" << _data->command << std::endl;
    if (_data->command == "stand") 
    {
        return FSM_StateName::STAND;
    }
    else
    {
        return FSM_StateName::PASSIVE;
    }
}

void State_Passive::onExit(){
    std::cout << "PASSIVE onExit" << std::endl;
}

