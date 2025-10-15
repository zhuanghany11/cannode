#include "valve_to_angle_controller.h"
#include <cmath>
#include <iostream>

namespace cannode {

ValveToAngleController::ValveToAngleController()
    : kp_(20.0), ki_(0.5), kd_(2.0),
      max_output_(1500.0), min_output_(0.0), max_integral_(500.0), deadzone_(0.5),
      error_(0.0), prev_error_(0.0), integral_(0.0) {
}

void ValveToAngleController::init(double kp, double ki, double kd, 
                                    double max_output, double min_output, 
                                    double max_integral, double deadzone) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_output_ = max_output;
    min_output_ = min_output;
    max_integral_ = max_integral;
    deadzone_ = deadzone;
    
    // 重置状态
    reset();
}

double ValveToAngleController::compute(double target_angle, double current_angle, double dt) {
    // 计算误差
    error_ = target_angle - current_angle;
    
    // 死区处理：如果误差在死区范围内，清零积分并返回0
    if (std::abs(error_) < deadzone_) {
        integral_ = 0.0;
        prev_error_ = 0.0;
        return 0.0;
    }
    
    // 计算比例项
    double p_term = kp_ * error_;
    
    // 计算积分项（带抗饱和）
    integral_ += error_ * dt;
    integral_ = clamp(integral_, -max_integral_, max_integral_);
    double i_term = ki_ * integral_;
    
    // 计算微分项
    double d_term = 0.0;
    if (dt > 0.0001) {  // 避免除零
        d_term = kd_ * (error_ - prev_error_) / dt;
    }
    
    // PID输出
    double output = p_term + i_term + d_term;
    
    // 输出限幅（确保非负，因为阀开度不能为负）
    output = clamp(output, min_output_, max_output_);
    
    // 更新历史误差
    prev_error_ = error_;
    
    return output;
}

void ValveToAngleController::reset() {
    error_ = 0.0;
    prev_error_ = 0.0;
    integral_ = 0.0;
}

} // namespace cannode

