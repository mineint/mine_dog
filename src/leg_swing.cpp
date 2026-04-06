#include "leg_swing.h"
#include "leg_kinematics.h"  
#include "leg_controller.h"  
#include <cmath>
#include <iostream>
#include <algorithm>  


LegSwingController::LegSwingController(const Eigen::Vector3f& leg_lengths) {
    kinematics_ = std::make_unique<LegKinematics>(leg_lengths);
    _hip_positions_body <<  0.15505f  , -0.209f , 0.0f ,
                            0.15505f  , 0.209f  , 0.0f ,
                            -0.15505f , -0.209f , 0.0f ,
                            -0.15505f ,  0.209f , 0.0f ; 
    _foot_positions_body.Zero();
    _foot_offset << 0.0f , 0.096f , 0.0f , 
                    0.0f , 0.096f  , 0.0f ,
                    0.0f , 0.096f  , 0.0f ,
                    0.0f , 0.096f , 0.0f ;  
    _foot_positions_leg.Zero();

}

// 主计算函数 
Eigen::Vector3d LegSwingController::computeSwingTorque(
    double t_normalized,
    double lift_height_,
    const Eigen::Vector3d& start_pos,
    const Eigen::Vector3d& end_pos,
    const Eigen::Vector3d& q,
    const Eigen::Vector3d& qd,
    const Eigen::Vector3d& joint_kp,
    const Eigen::Vector3d& joint_kd)
{
    t_normalized = std::clamp(t_normalized, 0.0, 1.0);
    tx_count ++;

    // 1. 生成期望足端位置 & 速度（使用五次贝塞尔）
    Eigen::Vector3d p_des = generateBezier5Trajectory(t_normalized, lift_height_, start_pos, end_pos);

    // 粗略速度估计（前后两点差分）
    constexpr double dt = 0.001;
    double t_next = std::min(t_normalized + dt, 1.0);
    Eigen::Vector3d p_next = generateBezier5Trajectory(t_next,  lift_height_, start_pos, end_pos);
    Eigen::Vector3d v_des = (p_next - p_des) / dt;

    last_foot_pos_ = p_des;
    last_foot_vel_ = v_des;

    static Eigen::Vector3d last_q_des = q; 
    Eigen::Vector3d q_des;
    try {
        q_des = inverseKinematics(p_des);
    } catch (...) { 
        return last_tau_; // 感觉有问题，后面注意一下 
    }

    double dt_actual = 0.001; 
    Eigen::Vector3d qd_des = (q_des - last_q_des) / dt_actual;
    last_q_des = q_des;

    // 关节空间 PD 控制
    Eigen::Vector3d q_err  = q_des - q;
    Eigen::Vector3d qd_err = qd_des - qd; 
    /* if (tx_count % 10 == 0)
        {
             std::cout << "q_des\n" << q_des << std::endl;
        } */

    Eigen::Vector3d tau = joint_kp.cwiseProduct(q_err) +
                          joint_kd.cwiseProduct(qd_err);

    last_tau_ = tau;

    /* if (tx_count % 10 == 0)
        {
             std::cout << "tau\n" << tau << std::endl;
        } */
    return tau;
}

// 五次贝塞尔轨迹
Eigen::Vector3d LegSwingController::generateBezier5Trajectory(
    double t,
    double lift_height_,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end) const
{
    double u  = t;
    double u2 = u * u;
    double u3 = u2 * u;
    double u4 = u3 * u;
    double u5 = u4 * u;

    double b0 = 1 - 5 * u + 10 * u2 - 10 * u3 + 5  * u4 - u5;
    double b1 = 5 * u - 20 * u2 + 30 * u3 - 20 * u4 + 5 * u5;
    double b2 = 10 * u2 - 30 * u3 + 30 * u4 - 10 * u5;
    double b3 = 10 * u3 - 20 * u4 + 10 * u5;
    double b4 = 5 * u4 - 5 * u5;
    double b5 = u5;

    // 计算总距离
    double dist = (end - start).norm();
    double h_offset = dist * 0.2; 


    Eigen::Vector3d dir = (end - start);
    dir.z() = 0; // 抹平高度差
    if (dir.norm() > 0.001) {
        dir.normalize(); // 变成单位向量
    } else {
        dir = Eigen::Vector3d::Zero(); // 原地踏步的情况
    }

    Eigen::Vector3d p0 = start;
    Eigen::Vector3d p1 = start + Eigen::Vector3d(0, 0, lift_height_ * 0.1);
    Eigen::Vector3d p2 = start + dir * h_offset + Eigen::Vector3d(0, 0, lift_height_ * 1.1);
    Eigen::Vector3d p3 = end - dir * h_offset + Eigen::Vector3d(0, 0, lift_height_ * 1.1);
    Eigen::Vector3d p4 = end + Eigen::Vector3d(0, 0, lift_height_ * 0.1);
    Eigen::Vector3d p5 = end;

    return b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3 + b4 * p4 + b5 * p5;
}

// ICR算法
void LegSwingController::ICR_compute(
    int leg_id,
    double rc_vx,
    double rc_vy,
    double rc_vw,
    double& target_x,
    double& target_y,
    double stance_time) 
    {
    
    rc_vx = std::clamp(rc_vx, -0.2, 0.2); // vx不要超过0.2
    rc_vy = std::clamp(rc_vy, -0.002, 0.002); // vy不要超过0.002

    ICR_vx[leg_id] = rc_vx - rc_vw * ICR_y[leg_id];
    ICR_vy[leg_id] = rc_vy + rc_vw * ICR_x[leg_id];
            
    target_x = ICR_vx[leg_id] * (stance_time / 2.0);
    double raw_target_y = ICR_vy[leg_id] * (stance_time / 2.0);

    // 处理Y轴：右侧腿翻转符号
    if (leg_id == 0 || leg_id == 2) {
        target_y = raw_target_y;
    } else {
        target_y = -raw_target_y;
    }
    
    }

// Raibert落脚点算法
Eigen::Vector3f LegSwingController::computeRaibertFootstep(
    int leg_id,
    Eigen::Vector3f desired_velocity,
    float stance_time,
    float swing_time,
    float step_height/* ,
    bool closed_gyro_z */) 
    {

    float prediction_time = stance_time + swing_time;
 
    float Vx = prediction_time * desired_velocity[0];
    float Vy = prediction_time * desired_velocity[1];
    
    // 从矩阵中获取当前腿的髋关节位置
    Eigen::Vector3f hip_position = _hip_positions_body.row(leg_id);
    float hip_offset_x = hip_position[0];
    float hip_offset_y = hip_position[1];
    
    float theta_0 = atan2(hip_offset_y, hip_offset_x);
    
    float R = sqrt(hip_offset_x * hip_offset_x + hip_offset_y * hip_offset_y);
    
    float theta_f = 0;

    // 暂不接入IMU
    /* if(closed_gyro_z) {
         theta_f = theta_0 + _estimator->_state_estimator_data.gyro_body[2] * prediction_time +
                 _k_feedback * (_estimator->_state_estimator_data.gyro_body[2] - desired_velocity[2]);
    } else {} */ 

    theta_f = theta_0 + desired_velocity[2] * prediction_time;
    
    
    float Xf = R * cos(theta_f);
    float Yf = R * sin(theta_f);
    
    float Xi = Vx + Xf;
    float Yi = Vy + Yf;
    
    // 存储机身坐标系下的足端位置
    _foot_positions_body(leg_id, 0) = Xi;
    _foot_positions_body(leg_id, 1) = Yi;
    _foot_positions_body(leg_id, 2) = step_height;
    
    // 转换为髋关节坐标系下（只计算当前腿）
    _foot_positions_leg.row(leg_id) = _foot_positions_body.row(leg_id) 
                                      - _hip_positions_body.row(leg_id) 
                                      + _foot_offset.row(leg_id);
    
    return _foot_positions_leg.row(leg_id);
}

// 逆运动学逆解算 
Eigen::Vector3d LegSwingController::inverseKinematics(const Eigen::Vector3d& p) const {
    return kinematics_->inverseKinematics(p);
}