#include "dog_fsm.h"
#include "leg_controller.h"
#include "state_jump.h"



void State_Jump::onEnter(){
    std::cout << "State Jump onEnter" << std::endl;
    last_high = _data->start_high;
    _data->j_timer = 0; 
}

void State_Jump::runState(){ 
    // 1. 更新步态调度器
    _data->gait_scheduler->update(_data->dt);

    // 2. 准备所有腿的当前状态和命令
    std::vector<Eigen::Vector3d> leg_torques(4);  // 每条腿的3个力矩
    _data->tx_count++;

    
    
    

    if (_data->j_timer < jump_time) 
    {
        _data->j_timer += 0.001; 

        // 下蹲中
        if (_data->j_timer < crouch_time)
        {   //std::cout << "下蹲中"<< std::endl;
            crouch_progress = _data->j_timer / crouch_time;
            double smooth_step = crouch_progress * crouch_progress * (3 - 2 * crouch_progress); 
            _data->start_high = last_high + smooth_step * (crouch_high - last_high);
            jump_pDes = {0.00, 0.096, _data->start_high};
        }
        
        // 阶段一飞行中（难点）
        else if (_data->j_timer < (crouch_time + flight_time_1))
        {
            double flight_progress_1 = (_data->j_timer - crouch_time) / flight_time_1;

            double smooth_step = flight_progress_1 * flight_progress_1 * (3 - 2 * flight_progress_1); 
            _data->start_high = crouch_high + smooth_step * (flight_high - crouch_high);
            _data->jump_long = 0 + smooth_step * (jump_back - 0);
            jump_pDes = {_data->jump_long, 0.096, _data->start_high};
            
        }

        else if (_data->j_timer < (crouch_time + flight_time - flight_time_2))
        {
            jump_pDes = flight_pos;
            
        }

        // 阶段二飞行中（难点）
        else if (_data->j_timer < (crouch_time + flight_time))
        {
            double flight_progress_2 = (_data->j_timer - crouch_time - flight_time + flight_time_2) / flight_time_2;

            double smooth_step = flight_progress_2 * flight_progress_2 * (3 - 2 * flight_progress_2); 
            _data->start_high = flight_high + smooth_step * (crouch_high - flight_high);
            _data->jump_long = jump_back + smooth_step * (0 - jump_back);
            jump_pDes = {_data->jump_long, 0.096, _data->start_high};
            
        }

        // 起立中
        else
        {
            double rise_progress = (_data->j_timer - crouch_time - flight_time) / crouch_time;
            double smooth_step = rise_progress * rise_progress * (3 - 2 * rise_progress);
            _data->start_high = crouch_high + smooth_step * (rise_high - crouch_high);
            jump_pDes = {0.00, 0.096, _data->start_high};
        }

    } 
        
    

    if (_data->tx_count % 50 == 0)
            { 
            //std::cout << "j_timer:" << _data->j_timer << std::endl;
            //std::cout << "crouch_progress:" << crouch_progress << std::endl;
            //std::cout << "start_high:" << _data->start_high << std::endl;
            std::cout << "jump_pDes:" << jump_pDes << std::endl;
            }
        
    // VMC相关参数
    LegCommand cmd;
    cmd.pDes   = jump_pDes;
    cmd.vDes   = Eigen::Vector3d::Zero();
    cmd.kpCart = _data->leg_controller->jump_kp;         
    cmd.kdCart = _data->leg_controller->jump_kd;

    for (int leg = 0; leg < 4; ++leg) {  
 

    Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(_data->leg_date[leg].q);
    Eigen::Matrix3d J = _data->kinematics->computeJacobian(_data->leg_date[leg].q);

    _data->legs_filter[leg].foot_vel = J * _data->leg_date[leg].qd;
    Eigen::Vector3d filtered_v = _data->legs_filter[leg].update_filter(_data->legs_filter[leg].foot_vel);  
       

    // 计算VMC力矩 
    auto leg_tau = _data->leg_controller->vmc_control(
        foot_pos, J , filtered_v,
        cmd.pDes, cmd.vDes,
        cmd.kpCart, cmd.kdCart 
    );             

    // 修正VMC力矩 
    const auto& signs = _data->leg_controller->leg_signs[leg];
    leg_tau(0) *= -signs.abad_sign;  
    leg_tau(1) *= -signs.hip_sign;  
    leg_tau(2) *= -signs.knee_sign;

    // 存储力矩
    leg_torques[leg] = leg_tau; 
}
    
    Eigen::VectorXd all_torques(12);
    int idx = 0;
    for (const auto& tau : leg_torques) {
        all_torques.segment<3>(idx) = tau;
        idx += 3;
    }
     
    _data->leg_controller->sendJointTorques(_data->can_ptr.get(), all_torques); 

    

 
}


FSM_StateName State_Jump::checkTransition(){

    //std::cout <<  "State_Jump检查切换" << _data->command << std::endl;
    if (_data->command == "passive")
    {
        return FSM_StateName::PASSIVE;
    }
    if (_data->command == "stand")
    {
        return FSM_StateName::STAND;
    }
    else
    {
        return FSM_StateName::JUMP;
    }
    
}

void State_Jump::onExit(){
    std::cout << "Stand Jump onExit" << std::endl;
}