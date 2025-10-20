#include "safety_monitor/safety_checker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sstream>

namespace safety_monitor
{

SafetyChecker::SafetyChecker(
  const std::array<std::array<double, 2>, 2> & position_limits,
  const std::array<double, 2> & torque_limits,
  const std::array<double, 2> & velocity_limits)
: position_limits_(position_limits),
  torque_limits_(torque_limits),
  velocity_limits_(velocity_limits)
{
  reset();
  RCLCPP_INFO(rclcpp::get_logger("SafetyChecker"), "安全检查器初始化完成");
}

void SafetyChecker::setSafetyLimits(
  const std::array<std::array<double, 2>, 2> & position_limits,
  const std::array<double, 2> & torque_limits,
  const std::array<double, 2> & velocity_limits)
{
  position_limits_ = position_limits;
  torque_limits_ = torque_limits;
  velocity_limits_ = velocity_limits;
  
  RCLCPP_INFO(rclcpp::get_logger("SafetyChecker"), "安全限制已更新");
}

bool SafetyChecker::performSafetyCheck(const sensor_msgs::msg::JointState & joint_state)
{
  // =================================================================
  // TODO: 核心安全检查逻辑区域
  // 这里需要实现具体的安全检查算法
  // =================================================================
  
  bool all_checks_passed = true;
  std::stringstream details;
  
  // 检查1: 关节位置安全
  bool position_safe = checkJointPositions(joint_state.position);
  if (!position_safe) {
    all_checks_passed = false;
    details << "关节位置超出安全范围; ";
  }
  
  // 检查2: 关节力矩安全
  bool torque_safe = checkJointTorques(joint_state.effort);
  if (!torque_safe) {
    all_checks_passed = false;
    details << "关节力矩超出限制; ";
  }
  
  // 检查3: 关节速度安全
  bool velocity_safe = checkJointVelocities(joint_state.velocity);
  if (!velocity_safe) {
    all_checks_passed = false;
    details << "关节速度异常; ";
  }
  
  // 检查4: 通信状态
  bool comm_safe = checkCommunicationStatus(joint_state);
  if (!comm_safe) {
    all_checks_passed = false;
    details << "通信状态异常; ";
  }
  
  last_check_details_ = details.str();
  
  if (all_checks_passed) {
    last_safe_time_ = rclcpp::Clock().now();
    RCLCPP_DEBUG(rclcpp::get_logger("SafetyChecker"), "所有安全检查通过");
  }
  
  return all_checks_passed;
}

std::string SafetyChecker::getLastCheckDetails() const
{
  return last_check_details_;
}

void SafetyChecker::reset()
{
  last_check_details_ = "安全检查器已重置";
  last_safe_time_ = rclcpp::Time(0, 0);
}

// =================================================================
// TODO: 以下是需要您实现的核心安全检查函数
// 请根据您的安全需求完善这些函数
// =================================================================

bool SafetyChecker::checkJointPositions(const std::vector<double> & positions)
{
  // TODO: 实现关节位置安全检查
  // 检查每个关节的位置是否在安全范围内
  
  if (positions.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节位置数据不完整");
    return false;
  }
  
  // 示例检查逻辑
  for (size_t i = 0; i < positions.size() && i < 2; ++i) {
    if (positions[i] < position_limits_[i][0] || positions[i] > position_limits_[i][1]) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), 
                 "关节%zu位置超出限制: %.3f not in [%.3f, %.3f]", 
                 i + 1, positions[i], position_limits_[i][0], position_limits_[i][1]);
      return false;
    }
  }
  
  return true;
}

bool SafetyChecker::checkJointTorques(const std::vector<double> & efforts)
{
  // TODO: 实现关节力矩安全检查
  // 检查每个关节的力矩是否超过安全限制
  
  if (efforts.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节力矩数据不完整");
    return false;
  }
  
  // 示例检查逻辑
  for (size_t i = 0; i < efforts.size() && i < 2; ++i) {
    if (std::abs(efforts[i]) > torque_limits_[i]) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), 
                 "关节%zu力矩超出限制: %.3f > %.3f", 
                 i + 1, std::abs(efforts[i]), torque_limits_[i]);
      return false;
    }
  }
  
  return true;
}

bool SafetyChecker::checkJointVelocities(const std::vector<double> & velocities)
{
  // TODO: 实现关节速度安全检查
  // 检查每个关节的速度是否异常
  
  if (velocities.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节速度数据不完整");
    return false;
  }
  
  // 示例检查逻辑
  for (size_t i = 0; i < velocities.size() && i < 2; ++i) {
    if (std::abs(velocities[i]) > velocity_limits_[i]) {
      RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), 
                 "关节%zu速度超出限制: %.3f > %.3f", 
                 i + 1, std::abs(velocities[i]), velocity_limits_[i]);
      return false;
    }
  }
  
  return true;
}

bool SafetyChecker::checkCommunicationStatus(const sensor_msgs::msg::JointState & joint_state)
{
  // TODO: 实现通信状态检查
  // 检查数据完整性、时间戳有效性等
  
  // 示例检查逻辑
  if (joint_state.name.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节名称数据不完整");
    return false;
  }
  
  if (joint_state.position.size() < 2) {
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "关节位置数据不完整");
    return false;
  }
  
  // 检查时间戳是否合理（不是太旧的数据）
  auto now = rclcpp::Clock().now();
  auto msg_time = rclcpp::Time(joint_state.header.stamp);
  auto time_diff = now - msg_time;
  
  if (time_diff.seconds() > 5.0) {  // 5秒超时
    RCLCPP_WARN(rclcpp::get_logger("SafetyChecker"), "传感器数据时间戳过旧");
    return false;
  }
  
  return true;
}

}  // namespace safety_monitor