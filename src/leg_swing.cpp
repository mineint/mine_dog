#include "leg_swing.h"
#include "leg_kinematics.h"  
#include "leg_controller.h"  
#include <cmath>
#include <iostream>
#include <algorithm>  


LegSwingController::LegSwingController(const Eigen::Vector3f& leg_lengths) {
    kinematics_ = std::make_unique<LegKinematics>(leg_lengths);
}

// 主计算函数 
Eigen::Vector3d LegSwingController::computeSwingTorque(
    double t_normalized,
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
    Eigen::Vector3d p_des = generateBezier5Trajectory(t_normalized, start_pos, end_pos);

    // 粗略速度估计（前后两点差分）
    constexpr double dt = 0.001;
    double t_next = std::min(t_normalized + dt, 1.0);
    Eigen::Vector3d p_next = generateBezier5Trajectory(t_next, start_pos, end_pos);
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

    double dist = (end - start).norm();
    double h_offset = dist * 0.2; 
    

    Eigen::Vector3d p0 = start;
    Eigen::Vector3d p1 = start + Eigen::Vector3d(0, 0, lift_height_ * 0.1);
    Eigen::Vector3d p2 = start + Eigen::Vector3d(h_offset, 0, lift_height_ * 1.1);
    Eigen::Vector3d p3 = end   + Eigen::Vector3d(-h_offset, 0, lift_height_ * 1.1);
    Eigen::Vector3d p4 = end   + Eigen::Vector3d(0, 0, lift_height_ * 0.1);
    Eigen::Vector3d p5 = end;

    return b0 * p0 + b1 * p1 + b2 * p2 + b3 * p3 + b4 * p4 + b5 * p5;
}

// Raibert落脚点算法
Eigen::Vector3d LegSwingController::generateBezier5Trajectory(
    )
{
    
    
}

// 逆运动学逆解算 
Eigen::Vector3d LegSwingController::inverseKinematics(const Eigen::Vector3d& p) const {
    return kinematics_->inverseKinematics(p);
}