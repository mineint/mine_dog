#pragma once

#include "gait_scheduler.h"
#include "leg_controller.h" 
#include "leg_kinematics.h"
#include "leg_swing.h"
#include "imu_reader.h"
#include "rc.h"
#include "radar_data.h"

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

struct RC_Date
{
    float CH1;
    float CH2;
    float CH3;
    float CH4;
    float S1;
    float S2;
    float S3;
    float S4;
};

struct Radar_Data
{
    int16_t x_pos;
    int16_t y_pos;
    int16_t z_pos;
    int16_t yaw_pos;
};
struct FSM_Data {
    double controlMode;        // 控制模式
    double timer = 0;          // 站立计时
    double j_timer = 0;        // 跳跃计时
    std::string command;       // 模式转换标志
    double stand_time = 2.0;   // 高度变化时间
    double start_wide = 0.096;
    double start_high = -0.12;
    double last_high = -0.12;
    double set_high = -0.28;
    double slope_up_high = 0.08;  // 上斜坡变化高度
    double slope_run_high = 0.06; // 过斜坡变化高度
    double jump_long = 0.00;   
    double trot_long = 0;
    double trot_long_r = 0;
    double trot_long_l = 0;
    double trot_wide = 0;
    double trot_wide_r = 0;
    double trot_wide_l = 0;
    double lift_height = 0.08;  // 抬腿高度

    double rc_vx;
    double rc_vy;
    double rc_vw;

    double target_x[4]; 
    double target_y[4]; 
    double target_z[4]; 
    
    std::shared_ptr<Tangair_usb2can> can_ptr;
    std::shared_ptr<ImuReader> imu_ptr;
    std::unique_ptr<GaitScheduler> gait_scheduler;
    std::unique_ptr<LegController> leg_controller;
    std::unique_ptr<LegKinematics> kinematics;   
    std::unique_ptr<LegSwingController> swing_controller;
    std::unique_ptr<USBRCReceiver> rc_ptr;
    std::unique_ptr<RadarReceiver> radar_ptr;

    Leg_Date leg_date[4];
    LegController legs_filter[4];   //专门用于处理滤波
    IMU_Date imu_data;
    RC_Date rc_data;
    Radar_Data radar_data;

    bool slow_torques_swith = true;   // 力矩缓起开关
    bool slope_swith = false;   // 斜坡开关
    bool bridge_swith = false;  // 2号桥开关
    int slope_state = 0; // 斜坡状态，0 = 正常, 1 = 横过， 2 = 上坡

    double dt = 0.001;  // 控制周期
    Eigen::Vector3d last_touchdown_pos[4];
    Eigen::Vector3d next_foot_target[4];

    uint32_t tx_count = 0;
  
};