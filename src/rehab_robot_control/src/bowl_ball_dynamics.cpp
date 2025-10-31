/**
 * @file bowl_ball_dynamics.cpp
 * @brief 碗-小球康复训练动力学模型实现文件
 * 
 * 这个文件实现了康复机器人系统中的核心动力学算法，模拟虚拟碗和小球之间的物理交互，
 * 用于上肢康复训练中的触觉反馈和阻力训练。
 * 
 * @author Your Name
 * @date 2025-10-23
 * @version 1.0
 * 
 * @copyright Copyright (c) 2025 ..
 */

#include "rehab_robot_control/bowl_ball_dynamics.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rehab_robot_control
{

/**
 * @brief 碗-小球动力学模型构造函数
 * 
 * 初始化动力学模型的各项参数和状态变量，建立虚拟碗和小球的初始状态。
 * 这个模型用于模拟康复训练中患者与虚拟环境之间的力反馈交互。
 * 
 * @param bowl_radius 虚拟碗的半径（米），决定训练范围大小
 * @param stiffness 虚拟弹簧刚度系数（N/m），控制力反馈强度
 * @param damping 虚拟阻尼系数（N·s/m），影响运动的平滑性
 * 
 * @note 碗半径应大于零，刚度和阻尼系数根据具体康复需求调整
 */
BowlBallDynamics::BowlBallDynamics(double bowl_radius, double stiffness, double damping)
: bowl_radius_(bowl_radius), stiffness_(stiffness), damping_(damping)
{
    // =======================================================================
    // 初始化小球状态：位置和速度归零
    // =======================================================================
    ball_position_.x = 0.0;  // 小球X轴位置（米）
    ball_position_.y = 0.0;  // 小球Y轴位置（米）  
    ball_position_.z = 0.0;  // 小球Z轴位置（米）- 通常为0，表示二维平面
    
    ball_velocity_.x = 0.0;  // 小球X轴速度（米/秒）
    ball_velocity_.y = 0.0;  // 小球Y轴速度（米/秒）
    ball_velocity_.z = 0.0;  // 小球Z轴速度（米/秒）
    
    // =======================================================================
    // 初始化碗的状态：默认位于原点上方0.1米处
    // =======================================================================
    bowl_position_.x = 0.0;    // 碗中心X轴位置（米）
    bowl_position_.y = 0.0;    // 碗中心Y轴位置（米）
    bowl_position_.z = 0.1;    // 碗中心Z轴高度（米）- 略高于小球
    
    // 记录初始化完成日志
    RCLCPP_INFO(rclcpp::get_logger("BowlBallDynamics"), 
               "碗-小球动力学模型初始化完成: 半径=%.2fm, 刚度=%.1fN/m, 阻尼=%.1fN·s/m", 
               bowl_radius_, stiffness_, damping_);
}

/**
 * @brief 设置训练参数
 * 
 * 动态调整康复训练的参数，允许在训练过程中根据患者情况调整难度。
 * 这些参数直接影响力反馈的强度和训练范围。
 * 
 * @param stiffness 新的弹簧刚度系数（N/m）- 值越大阻力越强
 * @param damping 新的阻尼系数（N·s/m）- 值越大运动越平滑
 * @param bowl_radius 新的碗半径（米）- 值越大训练范围越大
 * 
 * @note 参数变更会立即生效，影响下一次updateDynamics调用
 */
void BowlBallDynamics::setTrainingParameters(double stiffness, double damping, double bowl_radius)
{
    stiffness_ = stiffness;    // 更新刚度系数
    damping_ = damping;        // 更新阻尼系数
    bowl_radius_ = bowl_radius; // 更新碗半径
    
    // 可以添加参数验证逻辑，确保参数在合理范围内
}

/**
 * @brief 更新动力学模型的核心算法
 * 
 * 这是康复训练系统的核心算法，根据患者运动意图和当前关节状态，
 * 计算虚拟碗和小球的动力学行为，并生成相应的控制力矩。
 * 
 * 算法流程：
 * 1. 将真实关节位置映射为虚拟碗的位置
 * 2. 根据患者意图更新小球运动状态
 * 3. 检测小球与碗边的碰撞
 * 4. 计算虚拟力并映射为关节控制力矩
 * 
 * @param patient_intent 患者运动意图向量，表示患者想要推动的方向和力度
 * @param joint_positions 当前关节位置数组，用于确定虚拟碗的状态
 * @param dt 时间步长（秒），用于数值积分
 * 
 * @return 应施加于关节的控制力矩数组（Nm）
 * 
 * @warning 时间步长dt应保持稳定，避免数值积分误差积累
 */
std::array<double, 2> BowlBallDynamics::updateDynamics(
    const geometry_msgs::msg::Vector3 & patient_intent,
    const std::array<double, 2> & joint_positions,
    double dt)
{
    // =======================================================================
    // TODO: 实现您的碗-小球核心算法
    // 这里提供基础的算法框架，您需要根据具体的康复训练原理完善细节
    // =======================================================================
  
}

/**
 * @brief 获取当前碗的状态
 * 
 * 用于可视化或监控目的，返回虚拟碗的当前位姿信息。
 * 
 * @return 碗的位置向量（geometry_msgs::Vector3）
 * 
 * @note 返回的值是碗在虚拟空间中的坐标，可用于Unity可视化
 */
geometry_msgs::msg::Vector3 BowlBallDynamics::getBowlState() const
{
    return bowl_position_;  // 返回碗的当前位置
}

/**
 * @brief 获取当前小球的状态
 * 
 * 用于可视化或监控目的，返回虚拟小球的当前位姿信息。
 * 
 * @return 小球的位置向量（geometry_msgs::Vector3）
 * 
 * @note 返回的值是小球在虚拟空间中的坐标，可用于Unity可视化
 */
geometry_msgs::msg::Vector3 BowlBallDynamics::getBallState() const
{
    return ball_position_;  // 返回小球的当前位置
}

/**
 * @brief 重置动力学模型状态
 * 
 * 将碗和小球的状态恢复到初始位置，用于开始新的训练任务或系统复位。
 * 所有位置和速度变量归零，碗回到默认位置。
 * 
 * @note 这个方法不会改变训练参数（刚度、阻尼、碗半径）
 */
void BowlBallDynamics::reset()
{
    // 重置小球位置和速度
    ball_position_.x = 0.0;
    ball_position_.y = 0.0;
    ball_position_.z = 0.0;
    
    ball_velocity_.x = 0.0;
    ball_velocity_.y = 0.0;
    ball_velocity_.z = 0.0;
    
    // 重置碗的位置到默认值
    bowl_position_.x = 0.0;
    bowl_position_.y = 0.0;
    bowl_position_.z = 0.1;  // 默认高度0.1米
    
    // 可以添加重置日志记录
}

}  // namespace rehab_robot_control