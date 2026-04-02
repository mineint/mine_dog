#pragma once

#include <Eigen/Dense>
#include <memory>
#include <stdexcept>

class LegKinematics;  // 前向声明

struct LegState {
    Eigen::Vector3d q;   // 当前关节角度
    Eigen::Vector3d qd;  // 当前关节速度
};

class LegSwingController {
public:
    explicit LegSwingController(const Eigen::Vector3f& leg_lengths = Eigen::Vector3f(0.096f, 0.21f, 0.21f));

    // 主接口：计算摆动相关节力矩
    Eigen::Vector3d computeSwingTorque(
        double t_normalized,                    // [0,1]
        double lift_height_,
        const Eigen::Vector3d& start_pos,
        const Eigen::Vector3d& end_pos,
        const Eigen::Vector3d& q,
        const Eigen::Vector3d& qd,
        const Eigen::Vector3d& joint_kp  = Eigen::Vector3d(80.0, 80.0, 60.0),
        const Eigen::Vector3d& joint_kd  = Eigen::Vector3d(4.0,  4.0,  3.0)
    );

   
    // 五次贝塞尔轨迹生成（速度连续，适合摆动腿）
    Eigen::Vector3d generateBezier5Trajectory(
        double t,
        double lift_height_,
        const Eigen::Vector3d& start,
        const Eigen::Vector3d& end
    ) const;

    Eigen::Vector3f computeRaibertFootstep(
    int leg_id,
    Eigen::Vector3f desired_velocity,
    float stance_time,
    float swing_time,
    float step_height/* ,
    bool closed_gyro_z */);
private:

    std::unique_ptr<LegKinematics> kinematics_;

     

    mutable Eigen::Vector3d last_foot_pos_;
    mutable Eigen::Vector3d last_foot_vel_;
    Eigen::Vector3d last_tau_ = Eigen::Vector3d::Zero(); 
    Eigen::Vector3d last_q_des_ = Eigen::Vector3d::Zero();

    int tx_count = 0;


    Eigen::Matrix<float, 4, 3> _hip_positions_body;// 髋关节位置
    Eigen::Matrix<float, 4, 3> _foot_positions_body; // 足端位置 
    Eigen::Matrix<float, 4, 3> _foot_offset;// 足端位置在世界坐标系下
    Eigen::Matrix<float, 4, 3> _foot_positions_leg; // 足端位置在单腿坐标系下 [4x3]
    
    
    
    // 逆运动学
    Eigen::Vector3d inverseKinematics(const Eigen::Vector3d& p) const;
};