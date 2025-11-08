// 包含必要的头文件
#include "safety_monitor/safety_checker.hpp"  // 自定义安全检查器头文件，包含安全检查逻辑实现
#include <rclcpp/rclcpp.hpp>                  // ROS2 C++客户端库核心头文件，提供节点、发布者、订阅者等基础功能
#include <sensor_msgs/msg/joint_state.hpp>    // 传感器消息类型，用于传输关节状态（位置、速度、力矩等）
#include <std_msgs/msg/bool.hpp>              // 标准布尔消息类型，用于紧急停止信号传输
#include <std_srvs/srv/trigger.hpp>           // 标准服务类型，用于触发式服务通信（如手动紧急停止）
#include <memory>                             // C++智能指针库，用于自动内存管理

using namespace std::chrono_literals;         // 允许使用时间字面量（如100ms），简化时间参数设置

/**
 * @class SafetyMonitorNode
 * @brief 康复机器人安全监控节点类
 * 
 * 该节点是康复机器人系统的核心安全组件，负责实时监控机器人状态，
 * 在检测到异常（如超限、通信中断等）时自动触发紧急停止机制。
 * 基于ROS2框架实现，采用参数化配置、定时监控和多路通信机制。
 */
class SafetyMonitorNode : public rclcpp::Node
{
public:
  /**
   * @brief 节点构造函数
   * 
   * 初始化节点、声明参数、创建ROS2接口并启动安全监控循环。
   * 遵循ROS2节点生命周期管理规范[1](@ref)。
   */
  SafetyMonitorNode()
  : Node("safety_monitor_node")  // 调用基类构造函数，设置节点名称为"safety_monitor_node"
  {
    // 1. 参数管理：声明并获取所有配置参数
    declare_parameters();    // 声明节点参数（定义参数名称和默认值）
    get_parameters();        // 从参数服务器获取当前参数值
    
    // 2. 初始化安全检查器实例
    initialize_safety_checker();  // 创建安全检查器对象，注入参数配置
    
    // 3. 日志输出初始化信息（参数配置摘要）
    RCLCPP_INFO(get_logger(), "安全监控节点初始化完成");
    RCLCPP_INFO(get_logger(), "位置限制: Joint1[%.2f, %.2f], Joint2[%.2f, %.2f]",
                position_limits_[0][0], position_limits_[0][1],
                position_limits_[1][0], position_limits_[1][1]);
    RCLCPP_INFO(get_logger(), "力矩限制: [%.1f, %.1f] Nm", 
                torque_limits_[0], torque_limits_[1]);
    RCLCPP_INFO(get_logger(), "速度限制: [%.1f, %.1f] rad/s", 
                velocity_limits_[0], velocity_limits_[1]);
    
    // 4. 创建紧急停止信号发布者
    // 使用可靠QoS策略，确保紧急信号必达[9](@ref)
    emergency_stop_publisher_ = create_publisher<std_msgs::msg::Bool>(
      "/emergency_stop",                    // 主题名称：/emergency_stop
      rclcpp::QoS(10).reliable());          // 队列大小10，可靠性传输
    
    // 5. 创建传感器数据订阅者
    // 监听关节状态话题，异步接收实时数据[4](@ref)
    sensor_data_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/sensor_data",                       // 主题名称：/sensor_data
      10,                                   // 队列大小（缓存最新10条消息）
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        sensor_data_callback(msg);          // Lambda表达式绑定回调函数
      });
    
    // 6. 创建手动紧急停止服务
    // 提供外部触发紧急停止的接口[2](@ref)
    emergency_stop_service_ = create_service<std_srvs::srv::Trigger>(
      "/emergency_stop_service",            // 服务名称：/emergency_stop_service
      [this](const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
             std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        emergency_stop_service_callback(request, response);  // 服务请求回调
      });
    
    // 7. 创建安全监控定时器（核心监控循环）
    // 使用固定频率定时器，定期执行安全检查[9,10](@ref)
    safety_check_timer_ = create_wall_timer(
      100ms,                                // 周期100ms（10Hz监控频率）
      [this]() { safety_check_timer_callback(); });  // 定时器回调函数
    
    RCLCPP_INFO(get_logger(), "安全监控节点启动完成");
  }

private:
  /**
   * @brief 声明节点所有配置参数
   * 
   * 使用ROS2参数声明机制，定义参数名称、类型和默认值。
   * 支持动态参数更新和外部配置[6,7](@ref)。
   */
  void declare_parameters()
  {
    // 关节1位置限制（单位：弧度）
    declare_parameter("joint1.position_min", -3.14);  // 最小位置限制，默认-π
    declare_parameter("joint1.position_max", 3.14);   // 最大位置限制，默认π
    
    // 关节2位置限制（单位：弧度）
    declare_parameter("joint2.position_min", -2.0);   // 最小位置限制，默认-2.0
    declare_parameter("joint2.position_max", 2.0);     // 最大位置限制，默认2.0
    
    // 关节力矩限制（单位：牛顿米）
    declare_parameter("joint1.torque_limit", 50.0);  // 关节1最大力矩
    declare_parameter("joint2.torque_limit", 30.0);   // 关节2最大力矩
    
    // 关节速度限制（单位：弧度/秒）
    declare_parameter("joint1.velocity_limit", 3.0);  // 关节1最大速度
    declare_parameter("joint2.velocity_limit", 2.0);   // 关节2最大速度
    
    // 系统监控参数
    declare_parameter("safety_check_frequency", 10.0);  // 安全检查频率（Hz）
    declare_parameter("communication_timeout", 1.0);   // 通信超时阈值（秒）
  }
  
  /**
   * @brief 获取已声明的参数值
   * 
   * 从参数服务器读取当前参数值并存储到成员变量中。
   * 支持运行时参数动态更新[7](@ref)。
   */
  void get_parameters()
  {
    // 获取关节1位置限制
    position_limits_[0][0] = get_parameter("joint1.position_min").as_double();
    position_limits_[0][1] = get_parameter("joint1.position_max").as_double();
    
    // 获取关节2位置限制
    position_limits_[1][0] = get_parameter("joint2.position_min").as_double();
    position_limits_[1][1] = get_parameter("joint2.position_max").as_double();
    
    // 获取力矩限制
    torque_limits_[0] = get_parameter("joint1.torque_limit").as_double();
    torque_limits_[1] = get_parameter("joint2.torque_limit").as_double();
    
    // 获取速度限制
    velocity_limits_[0] = get_parameter("joint1.velocity_limit").as_double();
    velocity_limits_[1] = get_parameter("joint2.velocity_limit").as_double();
    
    // 获取通信超时阈值
    communication_timeout_ = get_parameter("communication_timeout").as_double();
  }
  
  /**
   * @brief 初始化安全检查器实例
   * 
   * 创建安全检查器对象，并传入参数配置。
   * 使用智能指针管理对象生命周期[1](@ref)。
   */
  void initialize_safety_checker()
  {
    // 使用std::make_unique创建安全检查器实例（C++14智能指针）
    safety_checker_ = std::make_unique<safety_monitor::SafetyChecker>(
      position_limits_,   // 位置限制参数
      torque_limits_,     // 力矩限制参数  
      velocity_limits_    // 速度限制参数
    );
  }
  
  /**
   * @brief 传感器数据回调函数
   * @param msg 关节状态消息共享指针
   * 
   * 处理接收到的传感器数据，更新最新关节状态和时间戳。
   * 采用异步回调机制，避免阻塞主线程[4](@ref)。
   */
  void sensor_data_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = msg;         // 更新最新关节状态缓存
    last_sensor_time_ = now();          // 记录最新数据接收时间
    
    // 调试级别日志：记录详细传感器数据（生产环境可关闭）
    RCLCPP_DEBUG(get_logger(), "收到传感器数据: 位置[%.3f, %.3f], 力矩[%.3f, %.3f]",
                 msg->position[0], msg->position[1],   // 关节位置
                 msg->effort[0], msg->effort[1]);      // 关节力矩
  }
  
  /**
   * @brief 安全监控定时器回调函数
   * 
   * 定时执行安全检查逻辑，包括通信超时检测和状态验证。
   * 采用10Hz固定频率监控[10](@ref)。
   */
  void safety_check_timer_callback()
  {
    // 1. 检查传感器通信超时
    if (is_communication_timeout()) {
      //  throttled日志：每5秒输出一次警告，避免日志洪泛
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, 
                          "传感器通信超时，触发紧急停止");
      trigger_emergency_stop("通信超时");  // 触发紧急停止
      return;  // 退出本次检查
    }
    
    // 2. 检查是否有有效传感器数据
    if (!latest_joint_state_) {
      RCLCPP_DEBUG(get_logger(), "等待传感器数据...");
      return;  // 数据未就绪，等待下次检查
    }
    
    // 3. 执行核心安全检查逻辑
    bool is_safe = safety_checker_->performSafetyCheck(*latest_joint_state_);
    
    if (!is_safe) {
      // 安全检查失败：记录错误并触发紧急停止
      std::string details = safety_checker_->getLastCheckDetails();
      RCLCPP_ERROR(get_logger(), "安全检查失败: %s", details.c_str());
      trigger_emergency_stop(details);  // 传递具体失败原因
    } else {
      // 系统状态正常：记录调试信息
      RCLCPP_DEBUG(get_logger(), "系统状态安全");
    }
  }
  
  /**
   * @brief 检查传感器通信是否超时
   * @return true表示通信已超时，需要触发紧急停止
   * 
   * 基于时间戳判断最后传感器数据是否在合理时间范围内。
   * 防止因通信中断导致监控失效[3](@ref)。
   */
  bool is_communication_timeout()
  {
    if (!last_sensor_time_.nanoseconds()) {
      return false;  // 尚未收到任何传感器数据，不视为超时
    }
    
    auto current_time = now();                      // 获取当前时间
    auto time_diff = current_time - last_sensor_time_;  // 计算时间差
    return time_diff.seconds() > communication_timeout_;  // 判断是否超时
  }
  
  /**
   * @brief 触发紧急停止流程
   * @param reason 停止原因描述，用于日志记录
   * 
   * 发布紧急停止信号，并记录停止事件。
   * 确保紧急信号可靠传输[9](@ref)。
   */
  void trigger_emergency_stop(const std::string & reason)
  {
    // 1. 构造紧急停止消息
    auto emergency_msg = std_msgs::msg::Bool();
    emergency_msg.data = true;  // 设置停止标志为true
    
    // 2. 发布紧急停止信号
    emergency_stop_publisher_->publish(emergency_msg);
    
    // 3. 记录错误日志（最高优先级）
    RCLCPP_ERROR(get_logger(), "紧急停止触发: %s", reason.c_str());
    
    // 4. 记录紧急停止事件信息（用于后续分析）
    last_emergency_time_ = now();  // 记录停止时间
    emergency_reason_ = reason;    // 记录停止原因
  }
  
  /**
   * @brief 手动紧急停止服务回调函数
   * @param request 服务请求对象（未使用）
   * @param response 服务响应对象
   * 
   * 处理外部手动触发的紧急停止请求。
   * 服务接口提供外部控制能力[2](@ref)。
   */
  void emergency_stop_service_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    (void)request;  // 显式忽略未使用参数，避免编译器警告
    
    // 触发紧急停止
    trigger_emergency_stop("手动触发");
    
    // 设置服务响应
    response->success = true;           // 操作成功标志
    response->message = "紧急停止已触发";  // 响应消息
    
    RCLCPP_INFO(get_logger(), "手动紧急停止服务调用成功");
  }

  // 安全检查器实例（核心业务逻辑）
  std::unique_ptr<safety_monitor::SafetyChecker> safety_checker_;
  
  // 参数存储数组（基于C++11 std::array）
  std::array<std::array<double, 2>, 2> position_limits_;  // 二维位置限制[[min,max],[min,max]]
  std::array<double, 2> torque_limits_;                    // 一维力矩限制[min,max]
  std::array<double, 2> velocity_limits_;                  // 一维速度限制[min,max]
  double communication_timeout_;                           // 通信超时阈值（秒）
  
  // 数据缓存区（状态保持）
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;  // 最新关节状态缓存
  rclcpp::Time last_sensor_time_;                      // 最后传感器数据接收时间
  rclcpp::Time last_emergency_time_;                   // 最后紧急停止时间
  std::string emergency_reason_;                        // 紧急停止原因描述
  
  // ROS2通信接口（基于智能指针管理）
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr emergency_stop_publisher_;      // 紧急停止发布者
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_data_subscription_;  // 传感器数据订阅者
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr emergency_stop_service_;       // 紧急停止服务
  rclcpp::TimerBase::SharedPtr safety_check_timer_;                                  // 安全监控定时器
};

/**
 * @brief 主函数 - ROS2节点入口点
 * @param argc 命令行参数个数
 * @param argv 命令行参数数组
 * @return int 程序退出码
 * 
 * 标准ROS2节点启动流程：初始化→创建节点→进入事件循环→清理资源[1](@ref)。
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // 初始化ROS2客户端库，解析命令行参数
  
  auto node = std::make_shared<SafetyMonitorNode>();  // 创建节点共享指针（自动内存管理）
  
  rclcpp::spin(node);  // 进入事件循环，处理回调（阻塞式，直到收到终止信号）
  
  rclcpp::shutdown();  // 清理ROS2资源，关闭节点
  return 0;
}