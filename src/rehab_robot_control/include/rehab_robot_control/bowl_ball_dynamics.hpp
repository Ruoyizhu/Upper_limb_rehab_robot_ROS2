#ifndef REHAB_ROBOT_CONTROL__BOWL_BALL_DYNAMICS_HPP_
#define REHAB_ROBOT_CONTROL__BOWL_BALL_DYNAMICS_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <array>

namespace rehab_robot_control
{

class BowlBallDynamics
{
public:
    BowlBallDynamics(double bowl_radius = 0.1, double stiffness = 100.0, double damping = 5.0);
    
    void setTrainingParameters(double stiffness, double damping, double bowl_radius);
    
    std::array<double, 2> updateDynamics(
        const geometry_msgs::msg::Vector3 & patient_intent,
        const std::array<double, 2> & joint_positions,
        double dt);
    
    geometry_msgs::msg::Vector3 getBowlState() const;
    geometry_msgs::msg::Vector3 getBallState() const;
    void reset();

private:
    double bowl_radius_;
    double stiffness_;
    double damping_;
    
    geometry_msgs::msg::Vector3 ball_position_;
    geometry_msgs::msg::Vector3 ball_velocity_;
    geometry_msgs::msg::Vector3 bowl_position_;
};

}  // namespace rehab_robot_control

#endif  // REHAB_ROBOT_CONTROL__BOWL_BALL_DYNAMICS_HPP_