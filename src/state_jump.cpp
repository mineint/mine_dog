#include "dog_fsm.h"
#include "leg_controller.h"
#include "State_jump.h"
#include "State_jump.h"




void State_Jump::onEnter(){
    std::cout << "State onEnter" << std::endl;
    
}

void State_Jump::runState(){ 
    // 1. 更新步态调度器
    _data->gait_scheduler->update(_data->dt);

    // 2. 准备所有腿的当前状态和命令
    std::vector<Eigen::Vector3d> leg_torques(4);  // 每条腿的3个力矩
    _data->tx_count++;

    
    
    // 记得清零time
    const double jump_time = 2.2;    // 跳跃总时长
    const double crouch_time = 1.0;   // 下蹲/起立时长
    const double flight_time = 0.2;   // 飞行时长
    const double crouch_high = -0.18;   // 下蹲高度
    const double rise_high = -0.18;   // 起立高度

    if (_data->j_timer < jump_time) 
    {
        _data->j_timer += 0.001; 
        double progress = _data->j_timer / jump_time;

        // 下蹲中
        if (_data->j_timer < crouch_time)
        {
            double crouch_progress = _data->j_timer / crouch_time;
            _data->jump_back = 0.00;
            _data->start_high = _data->start_high + crouch_progress * (crouch_high - _data->start_high);
        }
        
        // 飞行中（难点）
        else if (_data->j_timer < (crouch_time + flight_time))
        {
            double flight_progress = (_data->j_timer - crouch_time) / flight_time;
            _data->start_high = -0.06 + crouch_progress * (-0.28 - (-0.06));
        }

        // 起立中
        else
        {
            double rise_progress = (_data->j_timer - crouch_time - flight_time) / crouch_time;
            _data->jump_back = 0.00;
            _data->start_high = _data->start_high + crouch_progress * (rise_high - _data->start_high);
        }

    } 
        
    Eigen::Vector3d nominal_pDes(_data->jump_back, 0.096, _data->start_high); 

    if (_data->tx_count % 100 == 0)
            { 
            
            std::cout << "start_high:" << _data->start_high << std::endl;
            std::cout << "timer:" << _data->timer << std::endl;
            }
        
    // VMC相关参数
    LegCommand cmd;
    cmd.pDes   = nominal_pDes;
    cmd.vDes   = Eigen::Vector3d::Zero();  // 站立时目标速度为 0
    cmd.kpCart = _data->leg_controller->stance_kp;         
    cmd.kdCart = _data->leg_controller->stance_kd;

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
    std::cout << "Stand onExit" << std::endl;
}