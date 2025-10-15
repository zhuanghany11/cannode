#pragma once

#include <cstdint>
#include <algorithm>

namespace cannode {

/**
 * @brief 加速度到速度的PID控制器
 * 
 * 该控制器接收目标速度和当前速度，计算出合适的加速度指令
 * 用于车辆速度的闭环控制
 */
class AccToSpeedController {
public:
    /**
     * @brief 构造函数，使用默认参数初始化
     */
    AccToSpeedController();
    
    /**
     * @brief 初始化PID参数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param max_output 最大输出限幅
     * @param min_output 最小输出限幅
     * @param max_integral 积分限幅
     */
    void init(double kp, double ki, double kd, 
              double max_output, double min_output, double max_integral);
    
    /**
     * @brief 计算控制输出
     * @param target_speed 目标速度 (m/s)
     * @param current_speed 当前速度 (m/s)
     * @param dt 时间间隔 (s)
     * @return 计算得到的加速度指令 (m/s^2)
     */
    double compute(double target_speed, double current_speed, double dt);
    
    /**
     * @brief 重置控制器状态
     */
    void reset();
    
    /**
     * @brief 获取当前误差
     */
    double getError() const { return error_; }
    
    /**
     * @brief 获取积分项
     */
    double getIntegral() const { return integral_; }

private:
    // PID参数
    double kp_;           // 比例增益
    double ki_;           // 积分增益
    double kd_;           // 微分增益
    
    // 限幅参数
    double max_output_;   // 最大输出（加速度上限）
    double min_output_;   // 最小输出（加速度下限）
    double max_integral_; // 积分限幅
    
    // 状态变量
    double error_;        // 当前误差
    double prev_error_;   // 上一次误差
    double integral_;     // 积分累积
    
    // 辅助函数：限幅
    double clamp(double value, double min_val, double max_val) {
        return std::max(min_val, std::min(value, max_val));
    }
};

} // namespace cannode

