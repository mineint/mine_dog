#include "dog_fsm.h"
#include "state_passive.h"
#include "state_stand.h"
#include "state_trot.h"
#include "state_jump.h"
#include "Tangair_usb2can.h"
#include "leg_controller.h"
#include "gait_scheduler.h"
#include "imu_reader.h"


// 采用单腿局部坐标系一致性，向外为正

// 电机方向修正
const float abad_side_sign[4] = {-1.f, 1.f, 1.f, -1.f};
const float hip_side_sign[4] = {1.f, -1.f, 1.f, -1.f};
const float knee_side_sign[4] = {1.f, -1.f, 1.f, -1.f};

// 零点偏移参数
const float abad_offset[4] = {0.4857f, 0.4857f, 0.4857f, 0.4857f};
const float hip_offset[4] = {-0.9565f, -0.9565f, -0.9565f, -0.9565f};
const float knee_offset[4] = {2.5144f, 2.5144f, 2.5144f, 2.5144f};

FSM::FSM(std::shared_ptr<Tangair_usb2can> can)
{
  _data = std::make_unique<FSM_Data>();
  _data->can_ptr = can;
  _data->imu_ptr = std::make_unique<ImuReader>();;
  _data->rc_ptr = std::make_unique<USBRCReceiver>();
  _data->radar_ptr = std::make_unique<RadarReceiver>();
  _data->leg_controller = std::make_unique<LegController>();
  _data->kinematics = std::make_unique<LegKinematics>(Eigen::Vector3f(0.096f, 0.21f, 0.21f));
  _data->swing_controller = std::make_unique<LegSwingController>();
  _data->gait_scheduler = std::make_unique<GaitScheduler>();
  readerKey = new KeyboardReader();

  // 初始化电机数据结构
  for (int i = 0; i < 4; ++i)
  {
    _data->leg_date[i] = Leg_Date();
  }
  _data->leg_controller->sendZeroTorques(_data->can_ptr.get());
  current_state_ = std::make_unique<State_Passive>(_data.get());
}

// 更新电机信息，并进行零点偏移
void FSM::update_motor(Tangair_usb2can *can_ptr)
{

  USB2CAN_CAN_Bus_Struct Motor_recieve[4];

  Motor_recieve[0] = can_ptr->USB2CAN0_CAN_Bus_1;
  Motor_recieve[1] = can_ptr->USB2CAN0_CAN_Bus_2;
  Motor_recieve[2] = can_ptr->USB2CAN1_CAN_Bus_1;
  Motor_recieve[3] = can_ptr->USB2CAN1_CAN_Bus_2;

  for (int leg = 0; leg < 4; leg++)
  {
    // q: 关节位置
    _data->leg_date[leg].q(0) = Motor_recieve[leg].ID_1_motor_recieve.current_position_f * abad_side_sign[leg] + abad_offset[leg];
    _data->leg_date[leg].q(1) = Motor_recieve[leg].ID_2_motor_recieve.current_position_f * hip_side_sign[leg] + hip_offset[leg];
    _data->leg_date[leg].q(2) = Motor_recieve[leg].ID_3_motor_recieve.current_position_f * knee_side_sign[leg] + knee_offset[leg];

    // qd: 关节速度
    _data->leg_date[leg].qd(0) = Motor_recieve[leg].ID_1_motor_recieve.current_speed_f * abad_side_sign[leg];
    _data->leg_date[leg].qd(1) = Motor_recieve[leg].ID_2_motor_recieve.current_speed_f * hip_side_sign[leg];
    _data->leg_date[leg].qd(2) = Motor_recieve[leg].ID_3_motor_recieve.current_speed_f * knee_side_sign[leg];
  }

  // 电机限位保护
  if (
      _data->leg_date[0].q(0) > -1 && _data->leg_date[0].q(0) < 1.2 &&
      _data->leg_date[0].q(1) > -1.7 && _data->leg_date[0].q(1) < 1.5 &&
      _data->leg_date[0].q(2) > 1 && _data->leg_date[0].q(2) < 2.6 &&
      _data->leg_date[1].q(0) > -1 && _data->leg_date[1].q(0) < 1.2 &&
      _data->leg_date[1].q(1) > -1.7 && _data->leg_date[1].q(1) < 1.5 &&
      _data->leg_date[1].q(2) > 1 && _data->leg_date[1].q(2) < 2.6 &&
      _data->leg_date[2].q(0) > -1 && _data->leg_date[2].q(0) < 1.2 &&
      _data->leg_date[2].q(1) > -1.7 && _data->leg_date[2].q(1) < 1.5 &&
      _data->leg_date[2].q(2) > 1 && _data->leg_date[2].q(2) < 2.6 &&
      _data->leg_date[3].q(0) > -1 && _data->leg_date[3].q(0) < 1.2 &&
      _data->leg_date[3].q(1) > -1.7 && _data->leg_date[3].q(1) < 1.5 &&
      _data->leg_date[3].q(2) > 1 && _data->leg_date[3].q(2) < 2.6)
  {

    if (_data->tx_count % 200 == 0)
    {

      // std::cout << "正常" << std::endl;
      // std::cout << "position 1: " << _data->leg_date[0].qd(0) << std::endl;
      // std::cout << "position 2: " << _data->leg_date[0].qd(1) << std::endl;
      // std::cout << "position 3: " << _data->leg_date[0].qd(2) << std::endl;
      // std::cout << "position 4: " << _data->leg_date[1].q(0) << std::endl;
      // std::cout << "position 5: " << _data->leg_date[1].q(1) << std::endl;
      // std::cout << "position 6: " << _data->leg_date[1].q(2) << std::endl;
      // std::cout << "position 7: " << _data->leg_date[2].q(0) << std::endl;
      // std::cout << "position 8: " << _data->leg_date[2].q(1) << std::endl;
      // std::cout << "position 9: " << _data->leg_date[2].q(2) << std::endl;
      // std::cout << "position10: " << _data->leg_date[3].q(0) << std::endl;
      // std::cout << "position11: " << _data->leg_date[3].q(1) << std::endl;
      // std::cout << "position12: " << _data->leg_date[3].q(2) << std::endl;
    }
  }
  else
  {
    _data->command = "passive";
    _data->leg_controller->sendZeroTorques(_data->can_ptr.get());
    // _data->can_ptr->DISABLE_ALL_MOTOR(100);

    if (_data->tx_count % 200 == 0)
    {
      std::cout << "危险：已进入被动状态！！！" << std::endl;
      // std::cout << "position 1: " << _data->leg_date[0].q(0) << std::endl;
      // std::cout << "position 2: " << _data->leg_date[0].q(1) << std::endl;
      // std::cout << "position 3: " << _data->leg_date[0].q(2) << std::endl;
      // std::cout << "position 4: " << _data->leg_date[1].q(0) << std::endl;
      // std::cout << "position 5: " << _data->leg_date[1].q(1) << std::endl;
      // std::cout << "position 6: " << _data->leg_date[1].q(2) << std::endl;
      // std::cout << "position 7: " << _data->leg_date[2].q(0) << std::endl;
      // std::cout << "position 8: " << _data->leg_date[2].q(1) << std::endl;
      // std::cout << "position 9: " << _data->leg_date[2].q(2) << std::endl;
      // std::cout << "position10: " << _data->leg_date[3].q(0) << std::endl;
      // std::cout << "position11: " << _data->leg_date[3].q(1) << std::endl;
      // std::cout << "position12: " << _data->leg_date[3].q(2) << std::endl;
    }
  }
}

void FSM::update_RC(USBRCReceiver *rc_ptr)
{
  _data->rc_data.CH1 = ((float)rc_ptr->_RCData.CH1 - 50) / 100;
  _data->rc_data.CH2 = 0; //(float)rc_ptr->_RCData.CH2 - 50;
  _data->rc_data.CH3 = ((float)rc_ptr->_RCData.CH3 - 50) / 200;
  _data->rc_data.CH4 = ((float)rc_ptr->_RCData.CH4 - 50) / 250;
  _data->rc_data.S1 = rc_ptr->_RCData.S1;
  _data->rc_data.S2 = rc_ptr->_RCData.S2;
  _data->rc_data.S3 = rc_ptr->_RCData.S3;
  _data->rc_data.S4 = rc_ptr->_RCData.S4;

  // std::cout << "CH1: " << _data->rc_data.CH1 << std::endl;
  // std::cout << "CH2: " << _data->rc_data.CH2 << std::endl;
  // std::cout << "CH3: " << _data->rc_data.CH3 << std::endl;
  // std::cout << "CH4: " << _data->rc_data.CH4 << std::endl;
  // std::cout << "S1: " << _data->rc_data.S1 << std::endl;
  // std::cout << "S2: " << _data->rc_data.S2 << std::endl;
  // std::cout << "S3: " << _data->rc_data.S3 << std::endl;
  // std::cout << "S4: " << _data->rc_data.S4 << std::endl;
}

void FSM::update_radar(RadarReceiver *radar_ptr)
{
  _data->radar_data.x_pos = radar_ptr->_RadarData.x_pos;
  _data->radar_data.y_pos = radar_ptr->_RadarData.y_pos;
  _data->radar_data.z_pos = radar_ptr->_RadarData.z_pos;
  _data->radar_data.yaw_pos = radar_ptr->_RadarData.yaw_pos;

  // std::cout << "x_pos: " << _data->radar_data.x_pos << std::endl;
  // std::cout << "y_pos: " << _data->radar_data.y_pos << std::endl;
  // std::cout << "z_pos: " << _data->radar_data.z_pos << std::endl;
  // std::cout << "yaw_pos: " << _data->radar_data.yaw_pos << std::endl;
}

void FSM::update_imu(ImuReader *imu_ptr)
{
  _data->imu_data.pitch = imu_ptr->pitch;
  _data->imu_data.roll = imu_ptr->roll;
  _data->imu_data.yaw = imu_ptr->g_output_info.attitude.yaw;

  if (_data->tx_count % 100 == 0)
  {

  // std::cout << "pitch: " << _data->imu_data.pitch << std::endl;
  // std::cout << "roll: " << _data->imu_data.roll << std::endl;
  // std::cout << "yaw: " << _data->imu_data.yaw << std::endl;
  }
}

void FSM::remote_control()
{
  if (_data->rc_data.S2 == 1)
  {
    _data->command = "passive";
    // std::cout << "切换到被动模式" << std::endl;
  }
  else if (_data->rc_data.S2 == 0)
  {
    _data->command = "stand";
    // std::cout << "切换到站立模式" << std::endl;
    if (_data->rc_data.S3 != last_S3)
    {
      if (_data->rc_data.S3 == 0)
      {
        _data->last_high = _data->set_high;
        _data->set_high = -0.18;
        _data->timer = 1.0;
        std::cout << "已趴下:down" << std::endl;
      }
      else if (_data->rc_data.S3 == 1)
      {
        _data->last_high = _data->set_high;
        _data->set_high = -0.28;
        _data->timer = 1.0;
        std::cout << "已起立:up" << std::endl;
      }
      last_S3 = _data->rc_data.S3;
    }
  }
  else if (_data->rc_data.S2 == 3)
  {

    _data->command = "trot";
    // std::cout << "切换到行走模式" << std::endl;
    if (_data->rc_data.S3 != last_S3)
    {
      if (_data->rc_data.S3 == 3)
      {
        _data->lift_height = 0.05;
        std::cout << "下台阶:" << std::endl;
      }
      else if (_data->rc_data.S3 == 0)
      {
        _data->lift_height = 0.10;
        std::cout << "正常高度:" << std::endl;
      }
      else if (_data->rc_data.S3 == 1)
      {
        _data->lift_height = 0.15;
        std::cout << "上台阶:" << std::endl;
      }
      last_S3 = _data->rc_data.S3;
    }
  }
  else
  {

    _data->command = "passive";
    // std::cout << "切换到被动模式" << std::endl;
  }

}

void FSM::key_control(char rc)
{
    // 键盘按键到控制模式的映射
    switch (rc)
    {
    case 's':
    case 'S':

      _data->command = "stand";
      std::cout << "切换到站立模式" << std::endl;
      break;

    case 't':
    case 'T':

      _data->command = "trot";
      std::cout << "切换到行走模式" << std::endl;
      break;

    case 'j':
    case 'J':

      _data->command = "jump";
      std::cout << "切换到跳跃模式" << std::endl;
      break;
    case 'p':
    case 'P':
    case ' ':

      _data->command = "passive";
      std::cout << "切换到被动模式" << std::endl;
      break;

    case 'd':
    case 'D':
      _data->slow_torques_swith = false;
      _data->slope_swith = false; 
      _data->last_high = _data->set_high;
      _data->set_high = -0.20;
      _data->stand_time = 1.0;
      _data->timer = 0.0;
      std::cout << "已趴下:down" << std::endl;
      break;

    case 'u':
    case 'U':
      _data->slow_torques_swith = false;
      _data->slope_swith = false; 
      _data->last_high = _data->set_high;
      _data->set_high = -0.28;
      _data->stand_time = 1.0;
      _data->timer = 0.0;
      std::cout << "已起立:up" << std::endl;
      break;

    case 'r':
    case 'R':
      
      _data->rc_vx = 0.2;
      _data->rc_vy = 0;
      _data->rc_vw = 0;

      std::cout << "已跑步:run" << std::endl;
      break;

    case 'w':
    case 'W':
      
      _data->rc_vx = 0.1;
      _data->rc_vy = 0;
      _data->rc_vw = 0;

      std::cout << "已前进:ddd" << std::endl;
      break;

    case 'f':
    case 'F':
      
      _data->rc_vx = -0.1;
      _data->rc_vy = 0;
      _data->rc_vw = 0;

      std::cout << "已后退:fff" << std::endl;
      break;
      
    case 'x':
    case 'X':
      
      _data->rc_vx = 0;
      _data->rc_vy = 0;
      _data->rc_vw = 0;

      std::cout << "已踏步:step" << std::endl;
      break;

    case 'l':
    case 'L':
      
      _data->rc_vx = 0;
      _data->rc_vy = 0;
      _data->rc_vw = 0.3;

      std::cout << "已左转:" << std::endl;
      break;

    case ';':
    case ':':
      
      _data->rc_vx = 0;
      _data->rc_vy = 0;
      _data->rc_vw = -0.3;

      std::cout << "已右转:" << std::endl;
      break;

    case 'n':
    case 'N':
      
      _data->rc_vx = 0;
      _data->rc_vy = 0.08;
      _data->rc_vw = 0;

      std::cout << "左平移:" << std::endl;
      break;

    case 'm':
    case 'M':
      
      _data->rc_vx = 0;
      _data->rc_vy = -0.08;
      _data->rc_vw = 0;

      std::cout << "右平移:" << std::endl;
      break;

    case '8':

      _data->lift_height = 0.15;
      std::cout << "上台阶:" << std::endl;
      break;

    case '5':

      _data->lift_height = 0.10;
      std::cout << "正常高度:" << std::endl;
      break;

    case '2':
    
      _data->lift_height = 0.08;
      std::cout << "下台阶:" << std::endl;
      break;

    case '_':
    case '-':

     // 过斜坡
     _data->slope_swith = true;
     _data->slope_state = 1;
     break;
    
    case '+':
    case '=':
     
     // 上斜坡
     _data->slope_swith = true;
     _data->slope_state = 2;
     break;
    
    case '0':
     
     // 恢复正常
     _data->slope_swith = true; 
     _data->slope_state = 0;
     
     break;
    }
}

void FSM::update(double dt)
{
  //  if (!_data || !_data.get() || !current_state_) {
  //     std::cerr << "[FSM] update: invalid state or data\n";
  //     return;
  // }

  _data.get()->dt = dt;

  char rc = readerKey->readKey();

  // 更新电机数据
  update_motor(_data->can_ptr.get());

  // 更新IMU数据
  update_imu(_data->imu_ptr.get());

  // 更新遥控数据
  update_RC(_data->rc_ptr.get());

  // 更新步态
  _data.get()->gait_scheduler->update(dt);

  // 执行当前状态的核心行为
  current_state_->runState();

  // 遥控器控制
  // remote_control();

  if (rc != 0)
  {
    key_control(rc);
  }

  

  // 检查切换
  FSM_StateName next_name = current_state_->checkTransition();

  if (next_name != current_state_->currentStateName_)
  {
    // 退出旧状态
    current_state_->onExit();
    std::cout << "已退出旧状态" << std::endl;
    std::cout << "新状态:" << _data->command << std::endl;
    // 切换新状态
    switch (next_name)
    {
    case FSM_StateName::PASSIVE:
      current_state_ = std::make_unique<State_Passive>(_data.get());
      break;
    case FSM_StateName::STAND:
      current_state_ = std::make_unique<State_Stand>(_data.get());
      break;
    case FSM_StateName::TROT:
      current_state_ = std::make_unique<State_Trot>(_data.get());
      break;
    case FSM_StateName::JUMP:
      current_state_ = std::make_unique<State_Jump>(_data.get());
      break;
    }

    // 进入新状态
    current_state_->onEnter();
  }
}

void FSM::transitionTo(FSM_StateName next)
{
}
