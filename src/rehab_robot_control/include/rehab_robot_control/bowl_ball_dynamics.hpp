/**
 * @file bowl_ball_dynamics.hpp
 * @brief 碗-小球动力学模型头文件
 * 
 * 该文件定义了BowlBallDynamics类，用于实现康复机器人训练中的虚拟碗-小球动力学模型。
 * 该模型通过模拟碗与球的物理交互，为患者提供触觉力反馈，用于上肢康复训练。
 * 
 * @author Your Name
 * @date 2025-10-23
 * @version 1.0
 * 
 * @copyright Copyright (c) 2025 康复机器人项目组
 */

#ifndef REHAB_ROBOT_CONTROL__BOWL_BALL_DYNAMICS_HPP_
#define REHAB_ROBOT_CONTROL__BOWL_BALL_DYNAMICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <array>

namespace rehab_robot_control
{

/**
 * @brief 碗-小球动力学模型类
 * 
 * 这个类实现了康复训练中的虚拟碗和小球物理模型。通过模拟小球在碗内的运动，
 * 根据患者运动意图和当前关节位置，计算应施加的力反馈，并将虚拟力映射到关节控制指令。
 * 该模型支持实时调整训练参数（如碗的大小、刚度和阻尼），以适应不同的康复阶段。
 * 
 * 核心功能：
 * - 模拟碗的边界约束和小球动力学
 * - 根据患者意图更新小球运动状态
 * - 计算碗与小球碰撞产生的虚拟力
 * - 将虚拟力映射为关节力矩指令
 * 
 * @see 相关论文或文档链接（如有）
 */
class BowlBallDynamics
{
public:
    /**
     * @brief 构造函数，初始化碗-小球动力学模型
     * 
     * @param bowl_radius 虚拟碗的半径（米），控制训练范围，默认0.1米
     * @param stiffness 虚拟弹簧刚度系数（N/m），影响力反馈强度，默认100.0
     * @param damping 虚拟阻尼系数（N·s/m），影响运动平滑度，默认5.0
     * 
     * @note 刚度系数越大，患者感受到的阻力越强；阻尼系数越大，运动越平滑但响应延迟增加
     */
    BowlBallDynamics(double bowl_radius = 0.1, double stiffness = 100.0, double damping = 5.0);
    
    /**
     * @brief 设置训练参数，允许动态调整模型特性
     * 
     * @param stiffness 新的刚度系数（N/m），必须为正数
     * @param damping 新的阻尼系数（N·s/m），必须为非负数
     * @param bowl_radius 新的碗半径（米），必须为正数
     * 
     * @warning 在训练过程中调整参数可能导致力反馈突变，建议在静止状态下调用
     * 
     * @return 无返回值，但会更新内部模型参数
     */
    void setTrainingParameters(double stiffness, double damping, double bowl_radius);
    
    /**
     * @brief 更新动力学模型状态，计算控制指令
     * 
     * 这是核心算法函数，根据患者运动意图和当前关节位置，计算下一时刻的动力学状态，
     * 并返回关节控制指令。函数内部会：
     * 1. 将关节位置映射为虚拟碗的状态
     * 2. 根据患者意图更新小球运动学
     * 3. 检测碗与小球碰撞，计算虚拟力
     * 4. 将虚拟力映射为关节力矩
     * 
     * @param patient_intent 患者运动意图向量，表示期望的运动方向和强度
     *        - x: 左右方向意图（右为正）
     *        - y: 前后方向意图（前为正） 
     *        - z: 上下方向意图（上为正，可能未使用）
     * @param joint_positions 当前关节位置数组，通常包含两个关节的角度（弧度）
     *        - joint_positions[0]: 关节1位置
     *        - joint_positions[1]: 关节2位置
     * @param dt 时间步长（秒），用于数值积分，必须为正数
     * 
     * @return 关节控制指令数组，包含两个关节的力矩或目标位置
     *         - 返回值[0]: 关节1控制指令
     *         - 返回值[1]: 关节2控制指令
     * 
     * @throws 可能抛出数值计算异常（如除零错误），调用者应处理异常
     */
    std::array<double, 2> updateDynamics(
        const geometry_msgs::msg::Vector3 & patient_intent,
        const std::array<double, 2> & joint_positions,
        double dt);
    
    /**
     * @brief 获取当前碗的状态信息
     * 
     * @return 碗的当前位置和姿态向量
     *         - x: 碗中心X坐标（米）
     *         - y: 碗中心Y坐标（米）
     *         - z: 碗中心Z坐标（米）或碗的倾斜角度
     * 
     * @note 碗的位置通常由关节位置通过运动学映射得到
     */
    geometry_msgs::msg::Vector3 getBowlState() const;
    
    /**
     * @brief 获取当前小球的状态信息
     * 
     * @return 小球的位置和速度向量
     *         - x: 小球X坐标（米）
     *         - y: 小球Y坐标（米） 
     *         - z: 小球Z坐标（米）或速度信息
     * 
     * @note 小球状态用于可视化反馈和调试监控
     */
    geometry_msgs::msg::Vector3 getBallState() const;
    
    /**
     * @brief 重置动力学模型到初始状态
     * 
     * 将小球位置和速度重置为零，碗位置重置到初始位置。
     * 通常在训练开始、暂停恢复或检测到异常状态时调用。
     * 
     * @return 无返回值，但会重置内部状态变量
     */
    void reset();

private:
    double bowl_radius_;       ///< 虚拟碗的半径（米），定义训练范围边界
    double stiffness_;         ///< 虚拟弹簧刚度系数（N/m），控制力反馈强度
    double damping_;           ///< 虚拟阻尼系数（N·s/m），影响运动阻尼特性
    
    geometry_msgs::msg::Vector3 ball_position_;  ///< 小球当前位置向量（米）
    geometry_msgs::msg::Vector3 ball_velocity_;  ///< 小球当前速度向量（米/秒）
    geometry_msgs::msg::Vector3 bowl_position_;   ///< 碗当前位置和姿态向量
};

}  // namespace rehab_robot_control

#endif  // REHAB_ROBOT_CONTROL__BOWL_BALL_DYNAMICS_HPP_