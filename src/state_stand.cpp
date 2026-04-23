#include "dog_fsm.h"
#include "leg_controller.h"
#include "state_stand.h"
#include <array>

void State_Stand::onEnter(){
    std::cout << "State Stand onEnter" << std::endl;
    
}

void State_Stand::runState(){ 


    std::vector<Eigen::Vector3d> leg_torques(4);  // 每条腿的3个力矩
    //std::cout << "现在是站立状态\n " << std::endl;
    _data->tx_count++;
  
   

    // 力矩缓起
    if (_data->timer <= 1.0 && _data->slow_torques_swith) 
    {
        torques_progress = _data->timer / 1.0;
        torques_smooth_step = torques_progress * torques_progress * (3 - 2 * torques_progress);
        torques_smooth_step = std::min(torques_smooth_step, 1.0);
    }
    
    // 高度缓起
    if (_data->timer < _data->stand_time) 
    {
        _data->timer += 0.001; 
        progress = _data->timer / _data->stand_time;
        smooth_step = progress * progress * (3 - 2 * progress);
        _data->start_high = _data->last_high + smooth_step * (_data->set_high - _data->last_high);
    } 
    
    // 平地的足端位置
    if (!_data->slope_swith){
    nominal_pDes.fill(Eigen::Vector3d(0.0, 0.096, _data->start_high)); 
    }
    // if (_data->tx_count % 100 == 0)
    //         { 
    //         std::cout << "start_high:" << _data->start_high << std::endl;
    //         std::cout << "timer:" << _data->timer << std::endl;
    //         }


    for (int leg = 0; leg < 4; ++leg) {  
    
    // 斜坡平衡
    if (_data->slope_swith) {
    
    nominal_pDes[leg](0) = 0;
    nominal_pDes[leg](0) = 0.096;

    double target_z = _data->start_high;
    const double step_size = 0.0002; // 建议改为 speed * dt

    switch (_data->slope_state) {
        case 0:
            
            break;
        case 1: // 侧倾补偿
            if (leg == 0 || leg == 2) 
                target_z = _data->start_high + _data->slope_run_high;
            break;
        case 2: // 俯仰补偿
            if (leg == 0 || leg == 1) 
                target_z = _data->start_high + _data->slope_up_high;
            break;
    }
    // 统一更新高度
    _data->kinematics->approach(nominal_pDes[leg](2), target_z, step_size);

    }
    

    

    if (_data->tx_count % 100 == 0)
        { 
        // std::cout << "target_x" << leg + 1 << ":" <<  _data->target_x[leg] << std::endl;
        // std::cout << "target_y" << leg + 1 << ":" << _data->target_y[leg] << std::endl;

        std::cout << leg + 1 << ":" << std::endl;
        std::cout << "nominal_pDes: " <<  nominal_pDes[leg](2) << std::endl;

        } 

    // VMC相关参数
    LegCommand cmd;
    cmd.pDes   = nominal_pDes[leg];
    cmd.vDes   = Eigen::Vector3d::Zero();  
    cmd.kpCart = _data->leg_controller->stance_kp;         
    cmd.kdCart = _data->leg_controller->stance_kd;

    // VMC参数获取
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
    _data->target_z[0] = nominal_pDes[0](2);
    _data->target_z[1] = nominal_pDes[1](2);
    _data->target_z[2] = nominal_pDes[2](2);
    _data->target_z[3] = nominal_pDes[3](2);
    std::cout << "Stand Stand onExit" << std::endl;
}

