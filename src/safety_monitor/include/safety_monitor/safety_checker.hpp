#ifndef SAFETY_MONITOR__SAFETY_CHECKER_HPP_
#define SAFETY_MONITOR__SAFETY_CHECKER_HPP_

#include <sensor_msgs/msg/joint_state.hpp>
#include <array>
#include <rclcpp/rclcpp.hpp>

namespace safety_monitor
{

/**
 * @brief 康复机器人安全检查器
 * 
 * 这个类实现了对康复机器人系统的实时安全监控，
 * 包括关节限位、力矩限制、系统状态异常检测等。
 */
class SafetyChecker
{
public:
  /**
   * @brief 构造函数
   * @param position_limits 关节位置限制 [min, max]
   * @param torque_limits 关节力矩限制
   * @param velocity_limits 关节速度限制
   */
  SafetyChecker(
    const std::array<std::array<double, 2>, 2> & position_limits,
    const std::array<double, 2> & torque_limits,
    const std::array<double, 2> & velocity_limits);
  
  /**
   * @brief 设置安全参数
   * @param position_limits 新的位置限制
   * @param torque_limits 新的力矩限制
   * @param velocity_limits 新的速度限制
   */
  void setSafetyLimits(
    const std::array<std::array<double, 2>, 2> & position_limits,
    const std::array<double, 2> & torque_limits,
    const std::array<double, 2> & velocity_limits);
  
  /**
   * @brief 执行安全检查
   * @param joint_state 当前关节状态
   * @return 安全检查结果 (true=安全, false=危险)
   */
  bool performSafetyCheck(const sensor_msgs::msg::JointState & joint_state);
  
  /**
   * @brief 获取最后一次检查的详细结果
   * @return 包含各个检查项结果的字符串
   */
  std::string getLastCheckDetails() const;
  
  /**
   * @brief 重置安全检查器状态
   */
  void reset();

private:
  // =================================================================
  // TODO: 以下是需要您实现的核心安全检查逻辑
  // 请根据您的安全需求完善这些函数
  // =================================================================
  
  /**
   * @brief 检查关节位置是否在安全范围内
   * @param positions 关节位置数组
   * @return 位置检查结果
   */
  bool checkJointPositions(const std::vector<double> & positions);
  
  /**
   * @brief 检查关节力矩是否超过限制
   * @param efforts 关节力矩数组
   * @return 力矩检查结果
   */
  bool checkJointTorques(const std::vector<double> & efforts);
  
  /**
   * @brief 检查关节速度是否正常
   * @param velocities 关节速度数组
   * @return 速度检查结果
   */
  bool checkJointVelocities(const std::vector<double> & velocities);
  
  /**
   * @brief 检查系统通信状态
   * @param joint_state 关节状态消息
   * @return 通信状态检查结果
   */
  bool checkCommunicationStatus(const sensor_msgs::msg::JointState & joint_state);
  
  // 安全限制参数
  std::array<std::array<double, 2>, 2> position_limits_;
  std::array<double, 2> torque_limits_;
  std::array<double, 2> velocity_limits_;
  
  // 状态变量
  std::string last_check_details_;
  rclcpp::Time last_safe_time_;
};

}  // namespace safety_monitor

#endif  // SAFETY_MONITOR__SAFETY_CHECKER_HPP_