#include "dog_fsm.h"
#include "leg_controller.h"
#include "state_stand.h"


void State_Stand::onEnter(){
    std::cout << "State Stand onEnter" << std::endl;
    
}

void State_Stand::runState(){ 
    // 1. 更新步态调度器
    _data->gait_scheduler->update(_data->dt);

    // 2. 准备所有腿的当前状态和命令
    std::vector<Eigen::Vector3d> leg_torques(4);  // 每条腿的3个力矩
    //std::cout << "现在是站立状态\n " << std::endl;
    _data->tx_count++;

    // 站立时所有腿都应为 stance
    
        
    // 缓慢起步 
    
    
    const double stand_time = 2.3;    // 需要缓起步就清零time
    if (_data->timer <= 1.0) 
    {
        torques_progress = _data->timer / 1.0;
        torques_smooth_step = torques_progress * torques_progress * (3 - 2 * torques_progress);
    }
    if (_data->timer < stand_time) 
    {
        _data->timer += 0.001; 
        progress = _data->timer / stand_time;
        smooth_step = progress * progress * (3 - 2 * progress);
        _data->start_high = _data->last_high + smooth_step * (_data->set_high - _data->last_high);
    } 
        
    Eigen::Vector3d nominal_pDes(0.0, 0.096, _data->start_high); 
    
    

    // if (_data->tx_count % 100 == 0)
    //         { 
            
    //         std::cout << "start_high:" << _data->start_high << std::endl;
    //         std::cout << "timer:" << _data->timer << std::endl;
    //         }
        
    // VMC相关参数
    LegCommand cmd;
    cmd.pDes   = nominal_pDes;
    cmd.vDes   = Eigen::Vector3d::Zero();  // 站立时目标速度为 0
    cmd.kpCart = _data->leg_controller->stance_kp;         
    cmd.kdCart = _data->leg_controller->stance_kd;

    for (int leg = 0; leg < 4; ++leg) {  
 

    Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(_data->leg_date[leg].q);

    if (_data->tx_count % 100 == 0)
        { 
        // std::cout << "target_x" << leg + 1 << ":" <<  _data->target_x[leg] << std::endl;
        // std::cout << "target_y" << leg + 1 << ":" << _data->target_y[leg] << std::endl;

        // std::cout << leg + 1 << ":" <<  foot_pos << std::endl;
        std::cout << "torques_smooth_step: " <<  torques_smooth_step << std::endl;

        } 
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
    leg_torques[leg] = leg_tau * torques_smooth_step; 
}
    
    Eigen::VectorXd all_torques(12);
    int idx = 0;
    for (const auto& tau : leg_torques) {
        all_torques.segment<3>(idx) = tau;
        idx += 3;
    }
     

    _data->leg_controller->sendJointTorques(_data->can_ptr.get(), all_torques); 

    

 
}


FSM_StateName State_Stand::checkTransition(){

    //std::cout <<  "State_Stand检查切换" << _data->command << std::endl;
    if (_data->command == "passive")
    {
        return FSM_StateName::PASSIVE;
    }
    if (_data->command == "trot")
    {
        return FSM_StateName::TROT;
    }
    if (_data->command == "jump") 
    {
        return FSM_StateName::JUMP;
    }
    else
    {
        return FSM_StateName::STAND;
    }
    
}

void State_Stand::onExit(){
    std::cout << "Stand Stand onExit" << std::endl;
}
