#include "rehab_robot_control/bowl_ball_dynamics.hpp"
#include <rclcpp/rclcpp.hpp>

namespace rehab_robot_control
{

BowlBallDynamics::BowlBallDynamics(double bowl_radius, double stiffness, double damping)
: bowl_radius_(bowl_radius), stiffness_(stiffness), damping_(damping)
{
    // 初始化状态
    ball_position_.x = 0.0;
    ball_position_.y = 0.0;
    ball_position_.z = 0.0;
    
    ball_velocity_.x = 0.0;
    ball_velocity_.y = 0.0;
    ball_velocity_.z = 0.0;
    
    bowl_position_.x = 0.0;
    bowl_position_.y = 0.0;
    bowl_position_.z = 0.1;
    
    RCLCPP_INFO(rclcpp::get_logger("BowlBallDynamics"), "碗-小球动力学模型初始化完成");
}

void BowlBallDynamics::setTrainingParameters(double stiffness, double damping, double bowl_radius)
{
    stiffness_ = stiffness;
    damping_ = damping;
    bowl_radius_ = bowl_radius;
}

std::array<double, 2> BowlBallDynamics::updateDynamics(
    const geometry_msgs::msg::Vector3 & patient_intent,
    const std::array<double, 2> & joint_positions,
    double dt)
{
    // TODO: 实现您的碗-小球核心算法
    // 这里提供简单的示例实现
    
    // 将关节位置映射为碗的位置
    bowl_position_.x = joint_positions[0] * 0.5;
    bowl_position_.y = joint_positions[1] * 0.5;
    
    // 根据患者意图更新小球速度
    ball_velocity_.x = patient_intent.x * 0.1;
    ball_velocity_.y = patient_intent.y * 0.1;
    
    // 更新小球位置
    ball_position_.x += ball_velocity_.x * dt;
    ball_position_.y += ball_velocity_.y * dt;
    
    // 计算小球与碗中心的距离
    double dx = ball_position_.x - bowl_position_.x;
    double dy = ball_position_.y - bowl_position_.y;
    double distance = sqrt(dx*dx + dy*dy);
    
    // 简单的力反馈计算
    std::array<double, 2> torques = {0.0, 0.0};
    
    if (distance > bowl_radius_) {
        // 小球在碗外，产生恢复力
        double force = stiffness_ * (distance - bowl_radius_) + damping_ * sqrt(ball_velocity_.x*ball_velocity_.x + ball_velocity_.y*ball_velocity_.y);
        torques[0] = -dx / distance * force * 0.1;
        torques[1] = -dy / distance * force * 0.1;
    } else {
        // 小球在碗内，较小的力
        torques[0] = patient_intent.x * stiffness_ * 0.01;
        torques[1] = patient_intent.y * stiffness_ * 0.01;
    }
    
    return torques;
}

geometry_msgs::msg::Vector3 BowlBallDynamics::getBowlState() const
{
    return bowl_position_;
}

geometry_msgs::msg::Vector3 BowlBallDynamics::getBallState() const
{
    return ball_position_;
}

void BowlBallDynamics::reset()
{
    ball_position_.x = 0.0;
    ball_position_.y = 0.0;
    ball_position_.z = 0.0;
    
    ball_velocity_.x = 0.0;
    ball_velocity_.y = 0.0;
    ball_velocity_.z = 0.0;
    
    bowl_position_.x = 0.0;
    bowl_position_.y = 0.0;
    bowl_position_.z = 0.1;
}

}  // namespace rehab_robot_control