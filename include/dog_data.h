#pragma once

#include "gait_scheduler.h"
#include "leg_controller.h" 
#include "leg_kinematics.h"
#include "leg_swing.h"
#include "imu_reader.h"

struct Leg_Date
{
    Eigen::Vector3d q;
    Eigen::Vector3d qd;
};

struct IMU_Date
{
    float pitch;			
	float roll;
	float yaw;
};


struct FSM_Data {
    double controlMode;      // 控制模式
    double timer = 0;        // 站立计时
    double j_timer = 0;        // 跳跃计时
    std::string command;     // 模式转换标志
    double start_high = -0.06;
    double jump_back = -0.06;   // 跳跃时的x方向
    double trot_long = 0;
    double trot_long_r = 0;
    double trot_long_l = 0;
    double trot_wide = 0;
    double trot_wide_r = 0;
    double trot_wide_l = 0;
    
    std::shared_ptr<Tangair_usb2can> can_ptr;
    std::shared_ptr<ImuReader> imu_ptr;
    std::unique_ptr<GaitScheduler> gait_scheduler;
    std::unique_ptr<LegController> leg_controller;
    std::unique_ptr<LegKinematics> kinematics;   
    std::unique_ptr<LegSwingController> swing_controller;
    std::unique_ptr<ImuReader> imu_reader;
    Leg_Date leg_date[4];
    LegController legs_filter[4];   //专门用于处理滤波
    IMU_Date imu_data;


    

    
    double dt = 0.001;  // 控制周期
    Eigen::Vector3d last_touchdown_pos[4];
    Eigen::Vector3d next_foot_target[4];

     
    Eigen::Vector3d swing_kp{20, 24, 24};
    Eigen::Vector3d swing_kd{1, 1.2, 1.2};

    //Eigen::Vector3d swing_kp{3, 3, 3};
    //Eigen::Vector3d swing_kd{0.15, 0.15, 0.15};

    uint32_t tx_count = 0;
  
};