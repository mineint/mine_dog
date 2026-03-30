#pragma once

#include <array>
#include <Eigen/Dense>
#include <string>

struct FSM_Data;

enum class GaitType {
    STAND,          // 静止站立
    TROT,           // 对角小跑
};

enum class LegPhase {
    SUPPORT,
    SWING,
};

class GaitScheduler {
public:
    GaitScheduler();


    // 更新步态相位（主控制循环中每周期调用）
    void update(double dt);

    // 获取当前全局相位 [0, 1)
    double getGlobalPhase() const { return global_phase_; }

    // 获取某条腿的本地相位 [0, 1)
    double getLegPhase(int leg_id) const;

    // 获取某条腿的支撑相占比
    double getDutyFactor() const;

    // 判断某条腿当前是support还是swing
    LegPhase getLegPhaseType(int leg_id) const;

    // 设置步态类型（会自动设置相位偏移和 duty factor）
    void setGaitType(GaitType type);

    // 重置相位（例如从站立切换到行走时）
    void resetPhase() { global_phase_ = 0.0; }

    GaitType current_gait_ = GaitType::TROT;

private:
    

    double global_phase_ = 0.0;         // [0, 1)
    double cycle_time_   = 1;         // 一步周期（秒），trot 常用 0.4~0.6s
    double duty_factor_  = 1;         // 支撑相占比（0.5 表示 50% stance）

    // 每条腿的相位偏移（典型值在 setGaitType 中设置）
    std::array<double, 4> phase_offsets_{0.0, 0.0, 0.0, 0.0};

    // 根据步态类型设置默认参数
    void configureGaitParameters();
};
