/**
 * @file robot_control_node.cpp
 * @brief 康复机器人核心控制节点 - 实现碗-小球动力学算法
 * 
 * 该节点是康复机器人系统的"大脑"，负责：
 * 1. 接收传感器数据和患者运动意图
 * 2. 运行碗-小球动力学模型计算控制指令
 * 3. 发布控制命令到执行器
 * 4. 提供可视化数据用于监控
 * 
 * @author 
 */

#include "rehab_robot_control/bowl_ball_dynamics.hpp"  // 碗-小球动力学模型头文件
#include <rclcpp/rclcpp.hpp>                          // ROS 2核心库
#include <sensor_msgs/msg/joint_state.hpp>            // 关节状态消息
#include <geometry_msgs/msg/vector3.hpp>              // 3D向量消息（患者意图）
#include <std_msgs/msg/float64_multi_array.hpp>      // 浮点数数组消息（控制指令）
#include <geometry_msgs/msg/point_stamped.hpp>        // 带时间戳的点消息（可视化数据）

using namespace std::chrono_literals;  // 启用时间字面量（如100ms）

/**
 * @class RobotControlNode
 * @brief 康复机器人主控制节点类
 * 
 * 继承自rclcpp::Node，实现康复机器人的核心控制逻辑。
 * 集成碗-小球动力学模型，将患者意图转换为机器人控制指令。
 */
class RobotControlNode : public rclcpp::Node
{
public:
  /**
   * @brief 构造函数 - 初始化节点和所有组件
   * 
   * 负责：
   * 1. 声明和获取ROS参数
   * 2. 初始化碗-小球动力学模型
   * 3. 创建ROS发布者和订阅者
   * 4. 设置控制定时器
   */
  RobotControlNode() : Node("robot_control_node")  // 调用基类构造函数，设置节点名
  {
    // =================================================================
    // 参数声明和获取区域
    // =================================================================
    
    // 声明节点参数（带默认值）
    this->declare_parameter<double>("control_frequency", 100.0);  // 控制频率(Hz)
    this->declare_parameter<double>("stiffness", 100.0);         // 虚拟弹簧刚度系数
    this->declare_parameter<double>("damping", 5.0);              // 虚拟阻尼系数
    this->declare_parameter<double>("bowl_radius", 0.1);          // 虚拟碗半径(米)
    
    // 获取参数值
    double control_frequency = this->get_parameter("control_frequency").as_double();
    double stiffness = this->get_parameter("stiffness").as_double();
    double damping = this->get_parameter("damping").as_double();
    double bowl_radius = this->get_parameter("bowl_radius").as_double();
    
    // =================================================================
    // 动力学模型初始化区域
    // =================================================================
    
    // 创建碗-小球动力学模型实例
    dynamics_ = std::make_unique<rehab_robot_control::BowlBallDynamics>(
      bowl_radius, stiffness, damping);
    
    // 输出初始化信息到日志
    RCLCPP_INFO(this->get_logger(), "康复机器人控制节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "控制频率: %.1f Hz", control_frequency);
    RCLCPP_INFO(this->get_logger(), "刚度系数: %.1f", stiffness);
    RCLCPP_INFO(this->get_logger(), "阻尼系数: %.1f", damping);
    RCLCPP_INFO(this->get_logger(), "碗半径: %.3f", bowl_radius);
    
    // =================================================================
    // ROS 2接口创建区域 - 发布者
    // =================================================================
    
    // 创建控制指令发布者 - 向执行器发送控制命令
    control_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/control_cmd", 10);  // 话题名：/control_cmd，队列大小：10
    
    // 创建可视化数据发布者 - 用于RVIZ2或Unity界面显示
    visualization_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/visualization_data", 10);  // 话题名：/visualization_data，队列大小：10
    
    // =================================================================
    // ROS 2接口创建区域 - 订阅者
    // =================================================================
    
    // 创建传感器数据订阅者 - 接收关节状态信息
    sensor_data_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/sensor_data", 10,  // 话题名：/sensor_data，队列大小：10
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->sensorDataCallback(msg);  //  Lambda表达式回调
      });
    
    // 创建患者意图订阅者 - 接收患者运动意图
    patient_intent_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/patient_intent", 10,  // 话题名：/patient_intent，队列大小：10
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        this->patientIntentCallback(msg);  // Lambda表达式回调
      });
    
    // =================================================================
    // 控制定时器设置区域
    // =================================================================
    
    // 创建控制定时器，按指定频率执行控制循环
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),  // 计算周期
      [this]() { this->controlTimerCallback(); });  // 定时器回调函数
    
    RCLCPP_INFO(this->get_logger(), "康复机器人控制节点启动完成");
  }

private:
  /**
   * @brief 传感器数据回调函数
   * @param msg 接收到的关节状态消息
   * 
   * 处理来自Gazebo或真实硬件的传感器数据，更新当前关节状态。
   * 消息包含位置、速度、力矩等信息。
   */
  void sensorDataCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = msg;  // 缓存最新的传感器数据
    RCLCPP_DEBUG(this->get_logger(), "收到传感器数据: 位置[%.3f, %.3f]", 
                 msg->position[0], msg->position[1]);  // 调试级别日志
  }
  
  /**
   * @brief 患者意图回调函数
   * @param msg 接收到的患者意图向量
   * 
   * 处理来自Unity界面或测试节点的患者运动意图。
   * 向量分量表示患者在3D空间中的运动方向。
   */
  void patientIntentCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_patient_intent_ = msg;  // 缓存最新的患者意图
    RCLCPP_DEBUG(this->get_logger(), "收到患者意图: [%.3f, %.3f, %.3f]", 
                 msg->x, msg->y, msg->z);  // 调试级别日志
  }
  
  /**
   * @brief 控制定时器回调函数 - 核心控制循环
   * 
   * 按固定频率执行：
   * 1. 检查数据完整性
   * 2. 调用动力学模型计算控制指令
   * 3. 发布控制命令和可视化数据
   * 4. 记录调试信息
   */
  void controlTimerCallback()
  {
    // 数据完整性检查 - 确保有足够数据进行计算
    if (!latest_joint_state_ || !latest_patient_intent_) {
      // 使用节流警告避免日志泛滥（每5秒最多输出一次）
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "等待传感器数据或患者意图...");
      return;  // 数据不全，跳过本次控制循环
    }
    
    // 提取关节位置数据
    std::array<double, 2> joint_positions = {0.0, 0.0};  // 初始化默认值
    if (latest_joint_state_->position.size() >= 2) {
      joint_positions[0] = latest_joint_state_->position[0];  // 关节1位置
      joint_positions[1] = latest_joint_state_->position[1];  // 关节2位置
    }
    
    // =================================================================
    // 核心算法调用区域 - 碗-小球动力学计算
    // =================================================================
    
    // 调用动力学模型计算关节力矩
    auto joint_torques = dynamics_->updateDynamics(
      *latest_patient_intent_,    // 患者运动意图
      joint_positions,            // 当前关节位置
      1.0 / this->get_parameter("control_frequency").as_double()  // 时间步长
    );
    
    // 发布控制指令到执行器
    auto control_msg = std_msgs::msg::Float64MultiArray();
    control_msg.data = {joint_torques[0], joint_torques[1]};  // 填充力矩数据
    control_cmd_publisher_->publish(control_msg);
    
    // 发布可视化数据用于监控
    publishVisualizationData();
    
    RCLCPP_DEBUG(this->get_logger(), "发布控制力矩: [%.3f, %.3f]", 
                 joint_torques[0], joint_torques[1]);  // 调试级别日志
  }
  
  /**
   * @brief 发布可视化数据函数
   * 
   * 将碗-小球状态转换为可视化消息，供RVIZ2或Unity界面使用。
   * 包含小球在3D空间中的位置信息。
   */
  void publishVisualizationData()
  {
    auto viz_msg = geometry_msgs::msg::PointStamped();
    viz_msg.header.stamp = this->now();      // 设置当前时间戳
    viz_msg.header.frame_id = "base_link";   // 坐标系ID
    
    // 从动力学模型获取小球当前状态
    auto ball_state = dynamics_->getBallState();
    viz_msg.point.x = ball_state.x;  // 小球X坐标
    viz_msg.point.y = ball_state.y;  // 小球Y坐标
    viz_msg.point.z = ball_state.z;  // 小球Z坐标
    
    visualization_publisher_->publish(viz_msg);  // 发布可视化消息
    
    RCLCPP_DEBUG(this->get_logger(), "发布可视化数据: [%.3f, %.3f, %.3f]", 
                 ball_state.x, ball_state.y, ball_state.z);  // 调试级别日志
  }

  // =================================================================
  // 成员变量声明区域
  // =================================================================
  
  /**
   * @brief 碗-小球动力学模型实例
   * 
   * 使用std::unique_ptr管理，确保资源自动释放。
   * 实现康复训练的核心算法逻辑。
   */
  std::unique_ptr<rehab_robot_control::BowlBallDynamics> dynamics_;
  
  /**
   * @brief 数据缓存变量 - 存储最新接收到的消息
   */
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;      ///< 最新关节状态[2](@ref)
  geometry_msgs::msg::Vector3::SharedPtr latest_patient_intent_;   ///< 最新患者意图[2](@ref)
  
  /**
   * @brief ROS 2通信接口
   */
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_cmd_publisher_;     ///< 控制指令发布者[5](@ref)
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr visualization_publisher_;   ///< 可视化数据发布者[5](@ref)
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_data_subscription_;   ///< 传感器数据订阅者[5](@ref)
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr patient_intent_subscription_; ///< 患者意图订阅者[5](@ref)
  rclcpp::TimerBase::SharedPtr control_timer_;  ///< 控制循环定时器[5](@ref)
};

/**
 * @brief 主函数 - ROS 2节点入口点
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 * @return int 程序退出码
 * 
 * 负责：
 * 1. 初始化ROS 2上下文
 * 2. 创建控制节点实例
 * 3. 进入主事件循环
 * 4. 关闭ROS 2
 */
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);  // 初始化ROS 2上下文
  
  // 创建控制节点实例（使用std::make_shared进行安全的内存管理）
  auto node = std::make_shared<RobotControlNode>();
  
  rclcpp::spin(node);  // 进入主事件循环，处理ROS消息和回调
  
  rclcpp::shutdown();  // 关闭ROS 2上下文，清理资源
  return 0;  // 程序正常退出
}