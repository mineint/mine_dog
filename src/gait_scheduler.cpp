#include "gait_scheduler.h"
#include <algorithm>
#include <cmath>
#include <iostream>

GaitScheduler::GaitScheduler() {
    configureGaitParameters();  // 默认站立
}

void GaitScheduler::update(double dt) {
    // 判断是否为站立状态
    if (current_gait_ == GaitType::STAND) {
        global_phase_ = 0.0;
        return;
    }

    global_phase_ += ( dt / cycle_time_);
    if (global_phase_ >= 1.0) {
        global_phase_ -= 1.0;
    }
}

double GaitScheduler::getLegPhase(int leg_id) const {
    if (leg_id < 0 || leg_id >= 4) {
        return 0.0;
    }
    return std::fmod(global_phase_ + phase_offsets_[leg_id], 1.0);
}

double GaitScheduler::getDutyFactor() const {
    return duty_factor_;
}


LegPhase GaitScheduler::getLegPhaseType(int leg_id) const {
    double leg_phase = getLegPhase(leg_id);
    return (leg_phase < duty_factor_) ? LegPhase::SUPPORT : LegPhase::SWING;
}

void GaitScheduler::setGaitType(GaitType type) {
    current_gait_ = type;
    configureGaitParameters();
}

void GaitScheduler::configureGaitParameters() {
    switch (current_gait_) {
        case GaitType::STAND:
            cycle_time_   = 1.0;  
            duty_factor_  = 1.0;   
            phase_offsets_ = {0.0, 0.0, 0.0, 0.0};
            break;

        case GaitType::TROT:
            cycle_time_   = 1.00;  
            duty_factor_  = 0.50;   // stance_time = 0.5
            phase_offsets_ = {0.00, 0.50, 0.50, 0.00};  // 对角腿同步
            break;
    }
}




