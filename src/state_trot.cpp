#include "dog_fsm.h"
#include "leg_controller.h"
#include "state_trot.h"
#include "gait_scheduler.h"

void State_Trot::onEnter()
{

    std::cout << "Trot onEnter" << std::endl;
}

void State_Trot::runState()
{
    // std::cout << "Trot RUN" << std::endl;
    // 1. 更新步态调度器
    _data->gait_scheduler->update(_data->dt);

    // 2. 准备所有腿的当前状态和命令
    std::vector<Eigen::Vector3d> leg_torques(4); // 每条腿的3个力矩

    _data->tx_count++;
    // 3. 判断步态并分开处理支撑腿和摆动腿
    for (int leg = 0; leg < 4; ++leg)
    {

        LegPhase phase = _data->gait_scheduler->getLegPhaseType(leg);
        double leg_phase = _data->gait_scheduler->getLegPhase(leg);
        double duty = _data->gait_scheduler->getDutyFactor();

        Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(_data->leg_date[leg].q);

        // 键盘版
        _data->swing_controller->ICR_compute(
            leg,
            _data->rc_vx,
            _data->rc_vy,
            _data->rc_vw,
            _data->target_x[leg],
            _data->target_y[leg],
            0.5);

        // 遥控器版
        // _data->swing_controller->ICR_compute(
        //     leg,
        //     _data->rc_data.CH3,
        //     -_data->rc_data.CH4,
        //     -_data->rc_data.CH1,
        //     _data->target_x[leg],
        //     _data->target_y[leg],
        //     0.5);
        // stance_time给固定值，调整步态是注意修改

        // 左右腿分别管理
        Eigen::Vector3d starting_pDes_l(-_data->target_x[leg],
                                        0.096 + _data->target_y[leg],
                                        _data->start_high);

        Eigen::Vector3d starting_pDes_r(-_data->target_x[leg],
                                        0.096 + _data->target_y[leg],
                                        _data->start_high);

        Eigen::Vector3d destination_pDes_l(_data->target_x[leg],
                                           0.096 - _data->target_y[leg],
                                           _data->start_high);

        Eigen::Vector3d destination_pDes_r(_data->target_x[leg],
                                           0.096 - _data->target_y[leg],
                                           _data->start_high);

        if (leg == 0 || leg == 2)
        {
            _data->last_touchdown_pos[leg] = starting_pDes_l;
            _data->next_foot_target[leg] = destination_pDes_l;
        }
        else if (leg == 1 || leg == 3)
        {
            _data->last_touchdown_pos[leg] = starting_pDes_r;
            _data->next_foot_target[leg] = destination_pDes_r;
        }

        if (_data->tx_count % 100 == 0)
        {
            std::cout << "last_touchdown_pos" << leg + 1 << ":" << _data->last_touchdown_pos[leg] << std::endl;
            std::cout << "next_foot_target" << leg + 1 << ":" << _data->next_foot_target[leg] << std::endl;

            // std::cout << leg + 1 << ":" <<  foot_pos << std::endl;
        }

        switch (phase)
        {
        case LegPhase::SUPPORT: // 支撑腿处理
        {

            double t_stance = leg_phase / duty;
            t_stance = std::clamp(t_stance, 0.0, 1.0);

            Eigen::Vector3d support_pDes = _data->swing_controller->generateBezier5Trajectory(t_stance, _data->lift_height, _data->next_foot_target[leg], _data->last_touchdown_pos[leg]);
            Eigen::Vector3d nominal_pDes = Eigen::Vector3d(support_pDes[0], support_pDes[1], _data->start_high);

            // VMC相关参数
            LegCommand cmd;
            cmd.pDes = nominal_pDes;
            cmd.vDes = Eigen::Vector3d::Zero();
            cmd.kpCart = _data->leg_controller->trot_kp;
            cmd.kdCart = _data->leg_controller->trot_kd;

            Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(_data->leg_date[leg].q);
            Eigen::Matrix3d J = _data->kinematics->computeJacobian(_data->leg_date[leg].q);
            _data->legs_filter[leg].foot_vel = J * _data->leg_date[leg].qd;
            Eigen::Vector3d filtered_v = _data->legs_filter[leg].update_filter(_data->legs_filter[leg].foot_vel);

            // 计算VMC力矩
            auto leg_tau = _data->leg_controller->vmc_control(
                foot_pos, J, filtered_v,
                cmd.pDes, cmd.vDes,
                cmd.kpCart, cmd.kdCart);

            // 修正VMC力矩
            const auto &signs = _data->leg_controller->leg_signs[leg];
            leg_tau(0) *= -signs.abad_sign;
            leg_tau(1) *= -signs.hip_sign;
            leg_tau(2) *= -signs.knee_sign;

            // 存储力矩
            leg_torques[leg] = leg_tau;

            // if (_data->tx_count % 10 == 0)
            // {
            // std::cout << "SUPPORT:" << leg + 1 << std::endl;
            // std::cout << nominal_pDes << std::endl;
            // }

            break;
        }

        case LegPhase::SWING: // 摆动腿处理
        {
            double t_swing = (leg_phase - duty) / (1.0 - duty);
            t_swing = std::clamp(t_swing, 0.0, 1.0);

            // auto leg_tau  = _data->swing_controller->computeSwingTorque(
            //     t_swing,           // 当前摆动进度 [0,1]
            //     _data->lift_height,
            //     _data->last_touchdown_pos[leg],         // 抬腿起点
            //     _data->next_foot_target[leg],           // 目标落地点
            //     _data->leg_date[leg].q,
            //     _data->leg_date[leg].qd,
            //     _data->swing_kp,
            //     _data->swing_kd
            // );

            Eigen::Vector3d swing_des = _data->swing_controller->generateBezier5Trajectory(t_swing, _data->lift_height, _data->last_touchdown_pos[leg], _data->next_foot_target[leg]);

            // VMC相关参数
            LegCommand cmd;
            cmd.pDes = swing_des;
            cmd.vDes = Eigen::Vector3d::Zero();
            cmd.kpCart = _data->leg_controller->swing_kp;
            cmd.kdCart = _data->leg_controller->swing_kd;

            Eigen::Vector3d foot_pos = _data->kinematics->forwardKinematics(_data->leg_date[leg].q);
            Eigen::Matrix3d J = _data->kinematics->computeJacobian(_data->leg_date[leg].q);
            _data->legs_filter[leg].foot_vel = J * _data->leg_date[leg].qd;
            Eigen::Vector3d filtered_v = _data->legs_filter[leg].update_filter(_data->legs_filter[leg].foot_vel);

            // 计算VMC力矩
            auto leg_tau = _data->leg_controller->vmc_control(
                foot_pos, J, filtered_v,
                cmd.pDes, cmd.vDes,
                cmd.kpCart, cmd.kdCart);

            // if (_data->tx_count % 10 == 0)
            // {
            // std::cout << "SWING:" << leg + 1 << std::endl;
            // std::cout << swing_des << std::endl;
            // }
            // 修正VMC力矩
            const auto &signs = _data->leg_controller->leg_signs[leg];
            leg_tau(0) *= -signs.abad_sign;
            leg_tau(1) *= -signs.hip_sign;
            leg_tau(2) *= -signs.knee_sign;

            // 存储力矩
            leg_torques[leg] = leg_tau;

            break;
        }
        }
        /* double t_stance = leg_phase / duty;
        double t_swing = (leg_phase - duty) / (1.0 - duty);
        Eigen::Vector3f desired_velocity = {0.f, 0.f, 0.1f};
        Eigen::Vector3f Raibert[leg];
        Raibert[leg] = _data->swing_controller->computeRaibertFootstep(leg, desired_velocity, t_stance, t_swing, _data->start_high);
        if (_data->tx_count % 10 == 0)
            {
            std::cout << "Raibert:" << leg + 1 << Raibert[leg] << std::endl;
            }  */
    }

    /* if (_data->tx_count % 100 == 0)
            {

            std::cout << "leg_torques\n" << leg_torques[0] << std::endl;

            } */

    Eigen::VectorXd all_torques(12);
    int idx = 0;
    for (const auto &tau : leg_torques)
    {
        all_torques.segment<3>(idx) = tau;
        idx += 3;
    }

    _data->leg_controller->sendJointTorques(_data->can_ptr.get(), all_torques);
}

FSM_StateName State_Trot::checkTransition()
{
    // std::cout <<  "State_Trot检查切换" << _data->command << std::endl;
    if (_data->command == "stand")
    {
        return FSM_StateName::STAND;
    }
    if (_data->command == "passive")
    {
        return FSM_StateName::PASSIVE;
    }
    else
    {
        return FSM_StateName::TROT;
    }
}

void State_Trot::onExit()
{

    std::cout << "Trot onExit" << std::endl;
}
