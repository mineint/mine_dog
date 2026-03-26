#include "leg_kinematics.h"
#include <iostream>

LegKinematics::LegKinematics(const Eigen::Vector3f& leg_lengths)
    : leg_lengths_(leg_lengths)
{
    if (leg_lengths_(0) <= 0 || leg_lengths_(1) <= 0 || leg_lengths_(2) <= 0) {
        throw std::invalid_argument("Leg lengths must be positive");
    }
}

// 正向运动学
Eigen::Vector3d LegKinematics::forwardKinematics(const Eigen::Vector3d& q) const {
    const double L1 = leg_lengths_(0);
    const double L2 = leg_lengths_(1);
    const double L3 = leg_lengths_(2);

    const double q_abad = q(0);
    const double q_hip  = q(1);
    const double q_knee = q(2);
    const double q_hk   = q_hip + q_knee;

    const double s_abad = std::sin(q_abad);
    const double c_abad = std::cos(q_abad);
    const double s_hip  = std::sin(q_hip);
    const double c_hip  = std::cos(q_hip);  
    const double s_hk   = std::sin(q_hk);
    const double c_hk   = std::cos(q_hk);


    Eigen::Vector3d foot_pos;
    foot_pos.x() = L2 * s_hip + L3 * s_hk;                                 // 前后
    foot_pos.y() = L1 * c_abad + (L2 * c_hip + L3 * c_hk) * s_abad;        // 左右
    foot_pos.z() = L1 * s_abad - (L2 * c_hip + L3 * c_hk) * c_abad;       // 上下 


    return foot_pos;
}

// 雅可比矩阵
Eigen::Matrix3d LegKinematics::computeJacobian(const Eigen::Vector3d& q) const {
    const double L1 = leg_lengths_(0);
    const double L2 = leg_lengths_(1);
    const double L3 = leg_lengths_(2);

    const double q_abad = q(0);
    const double q_hip  = q(1);
    const double q_knee = q(2);
    const double q_hk   = q_hip + q_knee;

    const double s_abad = std::sin(q_abad);
    const double c_abad = std::cos(q_abad);
    const double s_hip  = std::sin(q_hip);
    const double c_hip  = std::cos(q_hip);
    const double s_hk   = std::sin(q_hk);
    const double c_hk   = std::cos(q_hk);

    Eigen::Matrix3d J;

    // ∂x/∂q
    J(0,0) = 0.0;
    J(0,1) = L2 * c_hip + L3 * c_hk;
    J(0,2) = L3 * c_hk;

    // ∂y/∂q
    J(1,0) = -L1 * s_abad + (L2 * c_hip + L3 * c_hk) * c_abad;
    J(1,1) = -(L2 * s_hip + L3 * s_hk) * s_abad;
    J(1,2) = -L3 * s_hk * s_abad;

    // ∂z/∂q
    J(2,0) = L1 * c_abad + (L2 * c_hip + L3 * c_hk) * s_abad;
    J(2,1) = (L2 * s_hip + L3 * s_hk) * c_abad;
    J(2,2) = L3 * s_hk * c_abad; 

 

    return J;
}

Eigen::Vector3d LegKinematics::computeFootVelocity(const Eigen::Vector3d& q,
                                                   const Eigen::Vector3d& qd) const {
    return computeJacobian(q) * qd;
}

// 逆向运动学
Eigen::Vector3d LegKinematics::inverseKinematics(const Eigen::Vector3d& p, 
                                                 bool prefer_knee_bend_forward) const {
    const double L1 = leg_lengths_(0);
    const double L2 = leg_lengths_(1);
    const double L3 = leg_lengths_(2);

    // --- 1. 计算 q_abad ---
    double r_sq = p.y() * p.y() + p.z() * p.z();
    double L1_sq = L1 * L1;
    
    if (r_sq < L1_sq) {
        throw std::runtime_error("IK: target position inside L1 offset sphere (unreachable)");
    }

    // R 是腿平面在 y-z 平面上的投影长度
    double R = std::sqrt(r_sq - L1_sq);
    
    // 根据 FK: y = L1*cos(q1) + R*sin(q1), z = L1*sin(q1) - R*cos(q1)
    // 这是一个典型的辅助角公式 A*sin(x) + B*cos(x) = C
    // 解法：q_abad = atan2(z, y) + atan2(R, L1) 
    // 或者用几何法：
    double q_abad = std::atan2(p.z(), p.x()) ; // 这行逻辑需要对齐你的坐标系定义
    // 精确匹配你给出的 FK 的解：
    q_abad = std::atan2(p.z() * L1 + p.y() * R, p.y() * L1 - p.z() * R);

    // --- 2. 计算 q_knee ---
    // 在腿部转动平面内，目标点的有效坐标为 (x, R)
    double x = p.x();
    double dist_sq = x * x + R * R;
    
    double cos_knee = (dist_sq - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
    
    if (std::abs(cos_knee) > 1.0 + 1e-6) {
        throw std::runtime_error("IK: target position out of reach");
    }
    cos_knee = std::clamp(cos_knee, -1.0, 1.0);

    double q_knee = std::acos(cos_knee);
    if (!prefer_knee_bend_forward) {
        q_knee = -q_knee;
    }

    // --- 3. 计算 q_hip ---
    // 利用公式：x = (L2 + L3*cos_q3)*sin_q2 + (L3*sin_q3)*cos_q2
    //          R = (L2 + L3*cos_q3)*cos_q2 - (L3*sin_q3)*sin_q2
    double s3 = std::sin(q_knee);
    double c3 = std::cos(q_knee);
    double a = L2 + L3 * c3;
    double b = L3 * s3;
    
    // 解线性方程组得到 q_hip
    // sin_q2 = (a*x - b*R) / (a^2 + b^2)
    // cos_q2 = (b*x + a*R) / (a^2 + b^2)
    double q_hip = std::atan2(a * x - b * R, b * x + a * R);

    return Eigen::Vector3d(q_abad, q_hip, q_knee);
}

