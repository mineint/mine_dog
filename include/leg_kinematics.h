#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <stdexcept>

class LegKinematics {
public:
    explicit LegKinematics(const Eigen::Vector3f& leg_lengths);


    // 正向运动学：关节角 → 足端位置（相对于髋关节原点）
    Eigen::Vector3d forwardKinematics(const Eigen::Vector3d& q) const;

    // 雅可比矩阵（足端速度 = J * qd）
    Eigen::Matrix3d computeJacobian(const Eigen::Vector3d& q) const;

    // 足端速度
    Eigen::Vector3d computeFootVelocity(const Eigen::Vector3d& q, const Eigen::Vector3d& qd) const;

    // 逆向运动学：足端位置 → 关节角
    // 返回 q = [q_abad, q_hip, q_knee]
    // throw std::runtime_error 如果不可达
    Eigen::Vector3d inverseKinematics(const Eigen::Vector3d& p,
                                      bool prefer_knee_bend_forward = true) const;
    
    void approach(double& current, double target, double step);

private:
    Eigen::Vector3f leg_lengths_;  // [L1(abad offset), L2(thigh), L3(calf)]

};