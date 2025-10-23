/**
 * @file safety_checker.cpp
 * @brief 康复机器人安全监控核心算法实现
 * @author Your Name
 * @date 2023-10-23
 * @version 1.0
 * 
 * @copyright Copyright (c) 2023, Your Company
 * 
 * @brief 该文件实现了SafetyChecker类，负责康复机器人系统的实时安全监控。
 *        包括关节位置、力矩、速度限制检查以及通信状态监控。
 *        当检测到异常时，会触发安全机制防止设备损坏或人员受伤。
 */

#include "safety_monitor/safety_checker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace safety_monitor
{

/**
 * @brief SafetyChecker构造函数，初始化安全监控器
 * 
 * @param position_limits 关节位置限制数组，每个关节包含[min, max]两个值
 * @param torque_limits 关节力矩限制数组，每个关节的最大允许力矩
 * @param velocity_limits 关节速度限制数组，每个关节的最大允许速度
 * 
 * @details 构造函数会初始化所有安全限制参数，并调用reset()方法重置内部状态。
 *          完成后会记录初始化日志信息。
 */
SafetyChecker::SafetyChecker(
  const std::array<std::array<double, 2>, 2> & position_limits,
  const std::array<double, 2> & torque_limits,
  const std::array<double, 2> & velocity_limits)
: position_limits_(position_limits),  ///< 初始化关节位置限制
  torque_limits_(torque_limits),      ///< 初始化关节力矩限制
  velocity_limits_(velocity_limits)   ///< 初始化关节速度限制
{
  reset();  // 重置内部状态变量
  RCLCPP_INFO(rclcpp::get_logger("SafetyChecker"), "安全检查器初始化完成");
}

/**
 * @brief 动态更新安全限制参数
 * 
 * @param position_limits 新的关节位置限制
 * @param torque_limits 新的关节力矩限制
 * @param velocity_limits 新的关节速度限制
 * 
 * @details 该方法允许在运行时动态调整安全阈值，适用于不同训练模式或患者能力变化的情况。
 *          更新后会记录新的参数值到日志中。
 */
void SafetyChecker::setSafetyLimits(
  const std::array<std::array<double, 2>, 2> & position_limits,
  const std::array<double, 2> & torque_limits,
  const std::array<double, 2> & velocity_limits)
{
  position_limits_ = position_limits;  ///< 更新位置限制
  torque_limits_ = torque_limits;    ///< 更新力矩限制
  velocity_limits_ = velocity_limits; ///< 更新速度限制
  
  RCLCPP_INFO(rclcpp::get_logger("SafetyChecker"), "安全限制已更新");
}

/**
 * @brief 执行完整的安全检查流程
 * 
 * @param joint_state 关节状态消息，包含位置、速度、力矩等传感器数据
 * @return bool 安全检查结果，true表示安全，false表示存在安全隐患
 * 
 * @details 这是核心安全检查方法，按顺序执行以下检查：
 *          1. 关节位置安全检查
 *          2. 关节力矩安全检查  
 *          3. 关节速度安全检查
 *          4. 通信状态检查
 *          所有检查通过才返回true，任一检查失败即返回false并记录详细原因。
 */
bool SafetyChecker::performSafetyCheck(const sensor_msgs::msg::JointState & joint_state)
{
  // =================================================================
  // TODO: 核心安全检查逻辑区域
  // 这里需要实现具体的安全检查算法
  // =================================================================
  
  bool all_checks_passed = true;  ///< 总体检查结果标志
  std::stringstream details;      ///< 用于构建详细检查结果信息
  
  // 检查1: 关节位置安全 - 验证关节角度是否在机械限位内
  bool position_safe = checkJointPositions(joint_state.position);
  if (!position_safe) {
    all_checks_passed = false;
    details << "关节位置超出安全范围; ";
  }
  
  // 检查2: 关节力矩安全 - 验证电机输出力矩是否超过安全阈值
  bool torque_safe = checkJointTorques(joint_state.effort);
  if (!torque_safe) {
    all_checks_passed = false;
    details << "关节力矩超出限制; ";
  }
  
  // 检查3: 关节速度安全 - 验证关节运动速度是否异常
  bool velocity_safe = checkJointVelocities(joint_state.velocity);
  if (!velocity_safe) {
    all_checks_passed = false;
    details << "关节速度异常; ";
  }
  
  // 检查4: 通信状态 - 验证传感器数据通信是否正常
  bool comm_safe = checkCommunicationStatus(joint_state);
  if (!comm_safe) {
    all_checks_passed = false;
    details << "通信状态异常; ";
  }
  
  // 保存本次检查的详细结果
  last_check_details_ = details.str();
  
  // 如果所有检查通过，更新最后安全时间戳
  if (all_checks_passed) {
    last_safe_time_ = rclcpp::Clock().now();
    RCLCPP_DEBUG(rclcpp::get_logger("SafetyChecker"), "所有安全检查通过");
  }
  
  return all_checks_passed;
}

/**
 * @brief 获取最后一次安全检查的详细描述
 * 
 * @return std::string 包含检查结果的详细描述字符串
 * 
 * @details 返回字符串包含所有失败检查的简要说明，用于日志记录或用户界面显示。
 */
std::string SafetyChecker::getLastCheckDetails() const
{
  return last_check_details_;
}

/**
 * @brief 重置安全检查器内部状态
 * 
 * @details 将内部状态变量重置为初始值，用于系统重启或清除错误状态。
 */
void SafetyChecker::reset()
{
  last_check_details_ = "安全检查器已重置";
  last_safe_time_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
}

// =================================================================
// TODO: 以下是需要您实现的核心安全检查函数
// 请根据您的安全需求完善这些函数
// =================================================================

/**
 * @brief 检查关节位置是否在安全范围内
 * 
 * @param positions 关节位置数组，包含所有关节的当前角度值
 * @return bool 位置检查结果，true表示安全，false表示超出限制
 * 
 * @details 该方法验证每个关节的当前位置是否在预设的[min, max]范围内。
 *          如果数据不完整或任何关节超出限制，返回false并记录警告日志。
 */
bool SafetyChecker::checkJointPositions(const std::vector<double> & positions)
{
  // TODO: 实现关节位置安全检查
  // 检查每个关节的位置是否在安全范围内
  
  // 验证数据完整性：至少需要2个关节的数据
  if (positions.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节位置数据不完整");
    return false;
  }
  
  // 遍历所有关节，检查位置限制
  for (size_t i = 0; i < positions.size() && i < 2; ++i) {
    // 检查当前位置是否低于下限或高于上限
    if (positions[i] < position_limits_[i][0] || positions[i] > position_limits_[i][1]) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), 
                 "关节%zu位置超出限制: %.3f not in [%.3f, %.3f]", 
                 i + 1, positions[i], position_limits_[i][0], position_limits_[i][1]);
      return false;
    }
  }
  
  return true;
}

/**
 * @brief 检查关节力矩是否超过安全限制
 * 
 * @param efforts 关节力矩数组，包含所有关节的当前力矩值
 * @return bool 力矩检查结果，true表示安全，false表示超出限制
 * 
 * @details 该方法验证每个关节的当前力矩是否超过预设的安全阈值。
 *          使用绝对值比较，确保正负方向的力矩都受到限制。
 */
bool SafetyChecker::checkJointTorques(const std::vector<double> & efforts)
{
  // TODO: 实现关节力矩安全检查
  // 检查每个关节的力矩是否超过安全限制
  
  // 验证数据完整性
  if (efforts.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节力矩数据不完整");
    return false;
  }
  
  // 检查每个关节的力矩值
  for (size_t i = 0; i < efforts.size() && i < 2; ++i) {
    // 使用绝对值比较，确保双向力矩都受限制
    if (std::abs(efforts[i]) > torque_limits_[i]) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), 
                 "关节%zu力矩超出限制: %.3f > %.3f", 
                 i + 1, std::abs(efforts[i]), torque_limits_[i]);
      return false;
    }
  }
  
  return true;
}

/**
 * @brief 检查关节速度是否正常
 * 
 * @param velocities 关节速度数组，包含所有关节的当前速度值
 * @return bool 速度检查结果，true表示安全，false表示异常
 * 
 * @details 该方法验证每个关节的当前速度是否在合理范围内，防止速度过高导致失控。
 */
bool SafetyChecker::checkJointVelocities(const std::vector<double> & velocities)
{
  // TODO: 实现关节速度安全检查
  // 检查每个关节的速度是否异常
  
  // 验证数据完整性
  if (velocities.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节速度数据不完整");
    return false;
  }
  
  // 检查每个关节的速度值
  for (size_t i = 0; i < velocities.size() && i < 2; ++i) {
    // 使用绝对值比较，确保双向速度都受限制
    if (std::abs(velocities[i]) > velocity_limits_[i]) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), 
                 "关节%zu速度超出限制: %.3f > %.3f", 
                 i + 1, std::abs(velocities[i]), velocity_limits_[i]);
      return false;
    }
  }
  
  return true;
}

/**
 * @brief 检查通信状态和数据完整性
 * 
 * @param joint_state 关节状态消息，包含头部时间戳和数据字段
 * @return bool 通信状态检查结果，true表示正常，false表示异常
 * 
 * @details 该方法验证传感器数据的完整性和时效性，包括：
 *          1. 关节名称数据完整性
 *          2. 关节位置数据完整性  
 *          3. 数据时间戳是否过时（超过5秒超时限制）
 */
bool SafetyChecker::checkCommunicationStatus(const sensor_msgs::msg::JointState & joint_state)
{
  // TODO: 实现通信状态检查
  // 检查数据完整性、时间戳有效性等
  
  // 检查关节名称数据完整性
  if (joint_state.name.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节名称数据不完整");
    return false;
  }
  
  // 检查关节位置数据完整性
  if (joint_state.position.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节位置数据不完整");
    return false;
  }
  
  // 检查时间戳是否合理（不是太旧的数据）
  auto now = rclcpp::Clock().now();                    ///< 当前时间
  auto msg_time = rclcpp::Time(joint_state.header.stamp); ///< 消息时间戳
  auto time_diff = now - msg_time;                    ///< 时间差值
  
  // 5秒超时检查：如果数据过于陈旧，认为通信异常
  if (time_diff.seconds() > 5.0) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "传感器数据时间戳过旧");
    return false;
  }
  
  return true;
}

}  // namespace safety_monitor