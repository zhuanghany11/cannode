#include "acc_to_speed_controller.h"
#include <iostream>

namespace cannode {

AccToSpeedController::AccToSpeedController()
    : kp_(1.0), ki_(0.1), kd_(0.05),
      max_output_(3.0), min_output_(-3.0), max_integral_(5.0),
      error_(0.0), prev_error_(0.0), integral_(0.0) {
}

void AccToSpeedController::init(double kp, double ki, double kd, 
                                  double max_output, double min_output, double max_integral) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
    max_output_ = max_output;
    min_output_ = min_output;
    max_integral_ = max_integral;
    
    // 重置状态
    reset();
}

double AccToSpeedController::compute(double target_speed, double current_speed, double dt) {
    // 计算误差
    error_ = target_speed - current_speed;
    
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
    
    // 输出限幅
    output = clamp(output, min_output_, max_output_);
    
    // 更新历史误差
    prev_error_ = error_;
    
    return output;
}

void AccToSpeedController::reset() {
    error_ = 0.0;
    prev_error_ = 0.0;
    integral_ = 0.0;
}

} // namespace cannode

