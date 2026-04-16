#pragma once

#include <Eigen/Dense>
#include <memory>
#include <array>
#include <vector>
#include <stdexcept>     // 用于 std::out_of_range
#include <iostream>      // 用于调试输出
#include <algorithm>     // 用于 std::clamp (C++17+)
#include "Tangair_usb2can.h"
#include "leg_kinematics.h"

struct LegCommand {
    Eigen::Vector3d pDes   = Eigen::Vector3d::Zero();   // 期望足端位置 (m)，相对于髋关节坐标系
    Eigen::Vector3d vDes   = Eigen::Vector3d::Zero();   // 期望足端速度 (m/s)
    
    Eigen::Vector3d kpCart = Eigen::Vector3d::Zero();  // 位置增益 (N/m)，z 通常更高
    Eigen::Vector3d kdCart = Eigen::Vector3d::Zero();   // 速度增益 (Ns/m)

   
};


class LegController {
public:
    
    struct LegConfig {
        float abad_sign;
        float hip_sign;
        float knee_sign;
    };

    std::array<LegConfig, 4> leg_signs = {{
    { -1.0f,  1.0f,  1.0f },   // LF 左前
    { 1.0f, -1.0f, -1.0f },   // RF 右前
    { 1.0f,  1.0f,  1.0f },   // LH 左后
    {-1.0f, -1.0f, -1.0f }    // RH 右后
}};
    
    // VMC相关系数
    Eigen::Vector3d trot_kp{1500, 1800, 2000};
    Eigen::Vector3d trot_kd{30, 30, 45};
    Eigen::Vector3d swing_kp{1500, 1800, 2000};
    Eigen::Vector3d swing_kd{30, 30, 45};
    // Eigen::Vector3d stance_kp{500, 500, 500};
    // Eigen::Vector3d stance_kd{10, 10, 10};
    // Eigen::Vector3d trot_kp{1500, 1500, 2000};
    // Eigen::Vector3d trot_kd{30, 30, 30}; 
    Eigen::Vector3d stance_kp{1000, 1000, 1333};
    Eigen::Vector3d stance_kd{20, 20, 20};
    Eigen::Vector3d jump_kp{500, 500, 500};
    Eigen::Vector3d jump_kd{10, 10, 10};
    


    Eigen::Vector3d update_filter(Eigen::Vector3d current_v) {
        double alpha = 0.1;
        Eigen::Vector3d filtered_v = alpha * current_v + (1.0 - alpha) * last_filtered_v;
        last_filtered_v = filtered_v;
        for (int i = 0; i < 3; ++i) {
        if (std::abs(filtered_v[i]) < 0.008) {
            filtered_v[i] = 0.0;
        }
    }
        return filtered_v;
    }



    LegController();

    ~LegController();


    Eigen::Vector3d vmc_control(
        const Eigen::Vector3d& foot_pos,   
        const Eigen::Matrix3d& J,
        const Eigen::Vector3d& filtered_v,
        const Eigen::Vector3d& pDes,        
        const Eigen::Vector3d& vDes,       
        const Eigen::Vector3d& kpCart,     
        const Eigen::Vector3d& kdCart   
    ) const;

    void sendJointTorques(Tangair_usb2can* can_ptr,
                          const Eigen::VectorXd& desired_torques);

    void sendZeroTorques(Tangair_usb2can* can_ptr);
    Eigen::Vector3d foot_vel;
    Eigen::Vector3d last_filtered_v;
    double f_gravity = 0.0f;
private:

    std::unique_ptr<LegKinematics> kinematics_; 
    uint32_t tx_count = 0;

};


