#不同的电机控制模式参数  一共5个参数

<!-- Velocity Control -->
USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 2;  // 2 rad/s
USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 20;

<!-- Position Control -->
USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 1.5;  // 1.5 radians
USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 50;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 5;

<!-- Torque Control -->
USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 5.0;  // 5 N·m
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 0;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 0;

<!-- Combined Control -->
USB2CAN0_CAN_Bus_1.ID_1_motor_send.position = 1.0;  // 1.0 radians
USB2CAN0_CAN_Bus_1.ID_1_motor_send.speed = 0.5;     // Max speed 0.5 rad/s
USB2CAN0_CAN_Bus_1.ID_1_motor_send.torque = 2.0;    // 2 N·m feedforward
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kp = 100;
USB2CAN0_CAN_Bus_1.ID_1_motor_send.kd = 10;