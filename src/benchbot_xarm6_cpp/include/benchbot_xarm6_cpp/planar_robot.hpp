
#ifndef PLANAR_ROBOT_HPP
#define PLANAR_ROBOT_HPP
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

namespace benchbot_xarm6
{
class PlanarRobot
    {
    public:
        PlanarRobot(rclcpp::Node::SharedPtr node, std::string robot_prefix, bool repub_jointstates);
        ~PlanarRobot();

        void publish_planar_robot();
        void publish_joints_robot();
        void publish_both();
        void set_planar_targets(double x, double y, double z, double rand_rot);
        void set_joints_targets(std::vector<std::string> name, std::vector<double> target);

    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr planar_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_publisher_;
        rclcpp::TimerBase::SharedPtr timer_;
        sensor_msgs::msg::JointState jointstates_;
        geometry_msgs::msg::Transform transform_;
        std::string robot_prefix_;
        bool repub_jointstates_ = false;
        bool publish_topic_ = false;

        rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
        void JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg);
    };
}
#endif