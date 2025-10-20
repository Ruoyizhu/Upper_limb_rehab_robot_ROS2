#include "safety_monitor/safety_checker.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <memory>

using namespace std::chrono_literals;

/**
 * @brief 康复机器人安全监控节点
 * 
 * 这个节点负责实时监控机器人系统的安全状态，
 * 在检测到异常时触发紧急停止机制。
 */
class SafetyMonitorNode : public rclcpp::Node
{
public:
  SafetyMonitorNode()
  : Node("safety_monitor_node")
  {
    // 声明和获取参数
    declare_parameters();
    get_parameters();
    
    // 初始化安全检查器
    initialize_safety_checker();
    
    RCLCPP_INFO(get_logger(), "安全监控节点初始化完成");
    RCLCPP_INFO(get_logger(), "位置限制: Joint1[%.2f, %.2f], Joint2[%.2f, %.2f]",
                position_limits_[0][0], position_limits_[0][1],
                position_limits_[1][0], position_limits_[1][1]);
    RCLCPP_INFO(get_logger(), "力矩限制: [%.1f, %.1f] Nm", 
                torque_limits_[0], torque_limits_[1]);
    RCLCPP_INFO(get_logger(), "速度限制: [%.1f, %.1f] rad/s", 
                velocity_limits_[0], velocity_limits_[1]);
    
    // 创建发布者 - 紧急停止信号
    emergency_stop_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/emergency_stop", rclcpp::QoS(10).reliable());
    
    // 创建订阅者 - 传感器数据
    sensor_data_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/sensor_data", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        sensor_data_callback(msg);
      });
    
    // 创建服务 - 手动紧急停止
    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/emergency_stop_service",
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        emergency_stop_service_callback(request, response);
      });
    
    // 创建安全监控定时器
    safety_check_timer_ = create_wall_timer(
      100ms,  // 10Hz安全检查频率
      [this]() { safety_check_timer_callback(); });
    
    RCLCPP_INFO(get_logger(), "安全监控节点启动完成");
  }

private:
  /**
   * @brief 声明节点参数
   */
  void declare_parameters()
  {
    // 关节位置限制 [min, max]
    declare_parameter("joint1.position_min", -3.14);
    declare_parameter("joint1.position_max", 3.14);
    declare_parameter("joint2.position_min", -2.0);
    declare_parameter("joint2.position_max", 2.0);
    
    // 关节力矩限制
    declare_parameter("joint1.torque_limit", 50.0);
    declare_parameter("joint2.torque_limit", 30.0);
    
    // 关节速度限制
    declare_parameter("joint1.velocity_limit", 3.0);
    declare_parameter("joint2.velocity_limit", 2.0);
    
    // 监控参数
    declare_parameter("safety_check_frequency", 10.0);
    declare_parameter("communication_timeout", 1.0);
  }
  
  /**
   * @brief 获取节点参数
   */
  void get_parameters()
  {
    // 获取位置限制
    position_limits_[0][0] = get_parameter("joint1.position_min").as_double();
    position_limits_[0][1] = get_parameter("joint1.position_max").as_double();
    position_limits_[1][0] = get_parameter("joint2.position_min").as_double();
    position_limits_[1][1] = get_parameter("joint2.position_max").as_double();
    
    // 获取力矩限制
    torque_limits_[0] = get_parameter("joint1.torque_limit").as_double();
    torque_limits_[1] = get_parameter("joint2.torque_limit").as_double();
    
    // 获取速度限制
    velocity_limits_[0] = get_parameter("joint1.velocity_limit").as_double();
    velocity_limits_[1] = get_parameter("joint2.velocity_limit").as_double();
    
    communication_timeout_ = get_parameter("communication_timeout").as_double();
  }
  
  /**
   * @brief 初始化安全检查器
   */
  void initialize_safety_checker()
  {
    safety_checker_ = std::make_unique<safety_monitor::SafetyChecker>(
      position_limits_, torque_limits_, velocity_limits_);
  }
  
  /**
   * @brief 传感器数据回调函数
   * @param msg 关节状态消息
   */
  void sensor_data_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = msg;
    last_sensor_time_ = now();
    
    RCLCPP_DEBUG(get_logger(), "收到传感器数据: 位置[%.3f, %.3f], 力矩[%.3f, %.3f]",
                 msg->position[0], msg->position[1],
                 msg->effort[0], msg->effort[1]);
  }
  
  /**
   * @brief 安全监控定时器回调
   */
  void safety_check_timer_callback()
  {
    // 检查通信超时
    if (is_communication_timeout()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                          "传感器通信超时，触发紧急停止");
      trigger_emergency_stop("通信超时");
      return;
    }
    
    if (!latest_joint_state_) {
      RCLCPP_DEBUG(get_logger(), "等待传感器数据...");
      return;
    }
    
    // 执行安全检查
    bool is_safe = safety_checker_->performSafetyCheck(*latest_joint_state_);
    
    if (!is_safe) {
      std::string details = safety_checker_->getLastCheckDetails();
      RCLCPP_ERROR(get_logger(), "安全检查失败: %s", details.c_str());
      trigger_emergency_stop(details);
    } else {
      RCLCPP_DEBUG(get_logger(), "系统状态安全");
    }
  }
  
  /**
   * @brief 检查通信是否超时
   * @return true如果通信超时
   */
  bool is_communication_timeout()
  {
    if (!last_sensor_time_.nanoseconds()) {
      return false;  // 尚未收到任何数据
    }
    
    auto current_time = now();
    auto time_diff = current_time - last_sensor_time_;
    return time_diff.seconds() > communication_timeout_;
  }
  
  /**
   * @brief 触发紧急停止
   * @param reason 停止原因
   */
  void trigger_emergency_stop(const std::string & reason)
  {
    auto emergency_msg = std_msgs::msg::Bool();
    emergency_msg.data = true;
    emergency_stop_publisher_->publish(emergency_msg);
    
    RCLCPP_ERROR(get_logger(), "紧急停止触发: %s", reason.c_str());
    
    // 记录紧急停止事件
    last_emergency_time_ = now();
    emergency_reason_ = reason;
  }
  
  /**
   * @brief 紧急停止服务回调
   */
  void emergency_stop_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // 避免未使用参数警告
    
    trigger_emergency_stop("手动触发");
    
    response->success = true;
    response->message = "紧急停止已触发";
    
    RCLCPP_INFO(get_logger(), "手动紧急停止服务调用成功");
  }

  // 安全检查器实例
  std::unique_ptr<safety_monitor::SafetyChecker> safety_checker_;
  
  // 参数存储
  std::array<std::array<double, 2>, 2> position_limits_;
  std::array<double, 2> torque_limits_;
  std::array<double, 2> velocity_limits_;
  double communication_timeout_;
  
  // 数据缓存
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  rclcpp::Time last_sensor_time_;
  rclcpp::Time last_emergency_time_;
  std::string emergency_reason_;
  
  // ROS 2接口
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_data_subscription_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;
  rclcpp::TimerBase::SharedPtr safety_check_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SafetyMonitorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}