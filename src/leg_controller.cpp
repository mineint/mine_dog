#include "leg_controller.h"  
#include "leg_swing.h"
#include "dog_fsm.h"
#include <iostream>
#include <memory> 
#include <array>   


LegController::LegController() {

    kinematics_ = std::make_unique<LegKinematics>(
        Eigen::Vector3f(0.096f, 0.21f, 0.21f)
        );

    std::cout << "[INFO] 初始化LegController配置." << std::endl;

    

}

LegController::~LegController() {
}

Eigen::Vector3d LegController::vmc_control(
    const Eigen::Vector3d& foot_pos,
    const Eigen::Matrix3d& J,
    const Eigen::Vector3d& filtered_v,
    const Eigen::Vector3d& pDes,
    const Eigen::Vector3d& vDes,
    const Eigen::Vector3d& kpCart,
    const Eigen::Vector3d& kdCart) const
{

    
    Eigen::Vector3d pos_error = foot_pos - pDes;
    Eigen::Vector3d vel_error = filtered_v - vDes;

    if (pos_error.norm() < 0.001) pos_error.setZero();
    if (vel_error.norm() < 0.01) vel_error.setZero();

    Eigen::Vector3d F_p = kpCart.cwiseProduct(pos_error);
    Eigen::Vector3d F_d = kdCart.cwiseProduct(vel_error);

    double max_damping_f = 50.0;  
    F_d = F_d.cwiseMax(-max_damping_f).cwiseMin(max_damping_f);

    Eigen::Vector3d F_desired = F_p + F_d;

    const double f_max = 200.0;
    F_desired = F_desired.cwiseMin(f_max).cwiseMax(-f_max);
 
    Eigen::Vector3d tau = J.transpose() * F_desired; 
 
    const double tau_max = 15.0; 
    tau = tau.cwiseMin(tau_max).cwiseMax(-tau_max);

    


    return tau;  
    //return Eigen::Vector3d::Zero();
}


/* 发送所有电机力矩 */
void LegController::sendJointTorques(Tangair_usb2can* can_ptr,
                                     const Eigen::VectorXd& desired_torques) {
    if (!can_ptr) {
        std::cerr << "[ERROR] sendJointTorques: can_ptr is null!\n";
        return;
    }

    if (desired_torques.size() != 12) {
        std::cerr << "[ERROR] sendJointTorques: torque vector size must be 12, got "
                  << desired_torques.size() << "\n";
        return;
    }

    USB2CAN_CAN_Bus_Struct* buses[4] = {
        &can_ptr->USB2CAN0_CAN_Bus_1,  // 右前腿 (LF)
        &can_ptr->USB2CAN0_CAN_Bus_2,  // 左前腿 (RF)
        &can_ptr->USB2CAN1_CAN_Bus_1,  // 左后腿 (LH)
        &can_ptr->USB2CAN1_CAN_Bus_2   // 右后腿 (RH)
    };

    const float MAX_TORQUE = 15.0f;  // 电机最大力矩

    for (int leg = 0; leg < 4; ++leg) {
        Eigen::Vector3f tau = desired_torques.segment<3>(leg * 3).cast<float>();
      
        // 限幅 + deadzone
        for (int j = 0; j < 3; ++j) {
            tau(j) = std::clamp(tau(j), -MAX_TORQUE, MAX_TORQUE);
            //if (std::abs(tau(j)) < 0.02f) tau(j) = 0.0f;  // 小扭矩归零，避免抖动
        }

        // 填充到对应 CAN 总线
        auto* bus = buses[leg];
        bus->ID_1_motor_send.torque = tau(0);  // abad
        bus->ID_2_motor_send.torque = tau(1);  // hip
        bus->ID_3_motor_send.torque = tau(2);  // knee 
       
        /*  std::cout << "tau 1:\n " << can_ptr->USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque << std::endl;
        std::cout << "tau 2:\n " << can_ptr->USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque << std::endl;
        std::cout << "tau 3:\n " << can_ptr->USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque << std::endl;
        std::cout << "tau 4:\n " << can_ptr->USB2CAN0_CAN_Bus_2.ID_1_motor_send.torque << std::endl;
        std::cout << "tau 5:\n " << can_ptr->USB2CAN0_CAN_Bus_2.ID_2_motor_send.torque << std::endl;
        std::cout << "tau 6:\n " << can_ptr->USB2CAN0_CAN_Bus_2.ID_3_motor_send.torque << std::endl;
        std::cout << "tau 7:\n " << can_ptr->USB2CAN1_CAN_Bus_1.ID_1_motor_send.torque << std::endl;
        std::cout << "tau 8:\n " << can_ptr->USB2CAN1_CAN_Bus_1.ID_2_motor_send.torque << std::endl;
        std::cout << "tau 9:\n " << can_ptr->USB2CAN1_CAN_Bus_1.ID_3_motor_send.torque << std::endl;
        std::cout << "tau10:\n " << can_ptr->USB2CAN1_CAN_Bus_2.ID_1_motor_send.torque << std::endl;
        std::cout << "tau11:\n " << can_ptr->USB2CAN1_CAN_Bus_2.ID_2_motor_send.torque << std::endl;
        std::cout << "tau12:\n " << can_ptr->USB2CAN1_CAN_Bus_2.ID_3_motor_send.torque << std::endl;  */
   }  
    
    //can_ptr->USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -1;
    // 测试力矩
    //can_ptr->USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 1;
    //can_ptr->USB2CAN0_CAN_Bus_2.ID_1_motor_send.torque = -1;
    /*can_ptr->USB2CAN1_CAN_Bus_1.ID_1_motor_send.torque = -2;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_1_motor_send.torque = 2;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = -2;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_2_motor_send.torque = 2;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_2_motor_send.torque = -2;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_2_motor_send.torque = 2;

     can_ptr->USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = -2;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_3_motor_send.torque = 2;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_3_motor_send.torque = -2;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_3_motor_send.torque = 2;  */


    can_ptr->USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_1_motor_send.kd = 0;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_1_motor_send.kd = 0;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_1_motor_send.kd = 0;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 0;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_2_motor_send.kd = 0;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_2_motor_send.kd = 0;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_2_motor_send.kd = 0;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 0;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_3_motor_send.kd = 0;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_3_motor_send.kd = 0;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_3_motor_send.kd = 0;
}

/* 发送零力矩 */
void LegController::sendZeroTorques(Tangair_usb2can* can_ptr) {
    if (!can_ptr) {
        std::cerr << "[LegController] sendZeroTorques: can_ptr is null\n";
        return;
    }

   
    can_ptr->USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 3;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_1_motor_send.kd = 3;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_1_motor_send.kd = 3;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_1_motor_send.kd = 3;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_2_motor_send.kd = 3;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_2_motor_send.kd = 3;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_2_motor_send.kd = 3;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_2_motor_send.kd = 3;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_3_motor_send.kd = 3;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_3_motor_send.kd = 3;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_3_motor_send.kd = 3;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_3_motor_send.kd = 3; 

    can_ptr->USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_1_motor_send.torque = 0;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_1_motor_send.torque = 0;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_1_motor_send.torque = 0;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_2_motor_send.torque = 0;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_2_motor_send.torque = 0;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_2_motor_send.torque = 0;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_2_motor_send.torque = 0;

    can_ptr->USB2CAN0_CAN_Bus_1.ID_3_motor_send.torque = 0;
    can_ptr->USB2CAN0_CAN_Bus_2.ID_3_motor_send.torque = 0;
    can_ptr->USB2CAN1_CAN_Bus_1.ID_3_motor_send.torque = 0;
    can_ptr->USB2CAN1_CAN_Bus_2.ID_3_motor_send.torque = 0;  

    
}

