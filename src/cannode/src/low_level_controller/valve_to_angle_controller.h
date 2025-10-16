#pragma once

#include <cstdint>
#include <algorithm>

namespace cannode {

/**
 * @brief 液压阀开度到角度的 PID 控制器
 *
 * 用途：输入目标角度与当前角度，输出阀开度（或电流）指令。
 */
class ValveToAngleController {
public:
    /** @brief 构造函数，使用默认参数初始化 */
    ValveToAngleController();
    
    /**
     * @brief 初始化 PID 参数
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param max_output 输出上限（最大阀开度/电流）
     * @param min_output 输出下限（最小阀开度/电流）
     * @param max_integral 积分限幅
     * @param deadzone 误差死区（度），在此范围内不输出
     */
    void init(double kp, double ki, double kd, 
              double max_output, double min_output, double max_integral, double deadzone = 0.5);
    
    /**
     * @brief 计算控制输出
     * @param target_angle 目标角度 (度)
     * @param current_angle 当前角度 (度)
     * @param dt 控制周期 (s)
     * @return 阀开度/电流 指令
     */
    double compute(double target_angle, double current_angle, double dt);
    
    /** @brief 重置控制器状态（误差/积分） */
    void reset();
    
    /** @brief 获取当前误差 */
    double getError() const { return error_; }
    /** @brief 获取积分项 */
    double getIntegral() const { return integral_; }
    /** @brief 设置死区范围 */
    void setDeadzone(double deadzone) { deadzone_ = deadzone; }

private:
    // PID参数
    double kp_;           // 比例增益
    double ki_;           // 积分增益
    double kd_;           // 微分增益
    
    // 限幅参数
    double max_output_;   // 最大输出（最大阀开度）
    double min_output_;   // 最小输出（最小阀开度）
    double max_integral_; // 积分限幅
    double deadzone_;     // 死区范围（度）
    
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

