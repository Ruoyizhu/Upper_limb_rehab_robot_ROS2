#include "rehab_robot_control/bowl_ball_dynamics.hpp"
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

using namespace std::chrono_literals;

class RobotControlNode : public rclcpp::Node
{
public:
  RobotControlNode() : Node("robot_control_node")
  {
    // 声明和获取参数
    this->declare_parameter<double>("control_frequency", 100.0);
    this->declare_parameter<double>("stiffness", 100.0);
    this->declare_parameter<double>("damping", 5.0);
    this->declare_parameter<double>("bowl_radius", 0.1);
    
    double control_frequency = this->get_parameter("control_frequency").as_double();
    double stiffness = this->get_parameter("stiffness").as_double();
    double damping = this->get_parameter("damping").as_double();
    double bowl_radius = this->get_parameter("bowl_radius").as_double();
    
    // 初始化碗-小球动力学模型
    dynamics_ = std::make_unique<rehab_robot_control::BowlBallDynamics>(
      bowl_radius, stiffness, damping);
    
    RCLCPP_INFO(this->get_logger(), "康复机器人控制节点初始化完成");
    RCLCPP_INFO(this->get_logger(), "控制频率: %.1f Hz", control_frequency);
    RCLCPP_INFO(this->get_logger(), "刚度系数: %.1f", stiffness);
    RCLCPP_INFO(this->get_logger(), "阻尼系数: %.1f", damping);
    RCLCPP_INFO(this->get_logger(), "碗半径: %.3f", bowl_radius);
    
    // 创建发布者
    control_cmd_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/control_cmd", 10);
    
    visualization_publisher_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
      "/visualization_data", 10);
    
    // 创建订阅者
    sensor_data_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/sensor_data", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        this->sensorDataCallback(msg);
      });
    
    patient_intent_subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "/patient_intent", 10,
      [this](const geometry_msgs::msg::Vector3::SharedPtr msg) {
        this->patientIntentCallback(msg);
      });
    
    // 创建控制定时器
    control_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / control_frequency)),
      [this]() { this->controlTimerCallback(); });
    
    RCLCPP_INFO(this->get_logger(), "康复机器人控制节点启动完成");
  }

private:
  void sensorDataCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    latest_joint_state_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "收到传感器数据: 位置[%.3f, %.3f]", 
                 msg->position[0], msg->position[1]);
  }
  
  void patientIntentCallback(const geometry_msgs::msg::Vector3::SharedPtr msg)
  {
    latest_patient_intent_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "收到患者意图: [%.3f, %.3f, %.3f]", 
                 msg->x, msg->y, msg->z);
  }
  
  void controlTimerCallback()
  {
    // 检查是否有足够的数据进行计算
    if (!latest_joint_state_ || !latest_patient_intent_) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                          "等待传感器数据或患者意图...");
      return;
    }
    
    // 提取关节位置
    std::array<double, 2> joint_positions = {0.0, 0.0};
    if (latest_joint_state_->position.size() >= 2) {
      joint_positions[0] = latest_joint_state_->position[0];
      joint_positions[1] = latest_joint_state_->position[1];
    }
    
    // 调用动力学模型计算控制指令
    auto joint_torques = dynamics_->updateDynamics(
      *latest_patient_intent_, joint_positions, 
      1.0 / this->get_parameter("control_frequency").as_double());
    
    // 发布控制指令
    auto control_msg = std_msgs::msg::Float64MultiArray();
    control_msg.data = {joint_torques[0], joint_torques[1]};
    control_cmd_publisher_->publish(control_msg);
    
    // 发布可视化数据
    publishVisualizationData();
    
    RCLCPP_DEBUG(this->get_logger(), "发布控制力矩: [%.3f, %.3f]", 
                 joint_torques[0], joint_torques[1]);
  }
  
  void publishVisualizationData()
  {
    auto viz_msg = geometry_msgs::msg::PointStamped();
    viz_msg.header.stamp = this->now();
    viz_msg.header.frame_id = "base_link";
    
    // 获取小球状态用于可视化
    auto ball_state = dynamics_->getBallState();
    viz_msg.point.x = ball_state.x;
    viz_msg.point.y = ball_state.y;
    viz_msg.point.z = ball_state.z;
    
    visualization_publisher_->publish(viz_msg);
    
    RCLCPP_DEBUG(this->get_logger(), "发布可视化数据: [%.3f, %.3f, %.3f]", 
                 ball_state.x, ball_state.y, ball_state.z);
  }

  // 动力学模型实例
  std::unique_ptr<rehab_robot_control::BowlBallDynamics> dynamics_;
  
  // 数据缓存
  sensor_msgs::msg::JointState::SharedPtr latest_joint_state_;
  geometry_msgs::msg::Vector3::SharedPtr latest_patient_intent_;
  
  // ROS 2接口
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr control_cmd_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr visualization_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sensor_data_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr patient_intent_subscription_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RobotControlNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}