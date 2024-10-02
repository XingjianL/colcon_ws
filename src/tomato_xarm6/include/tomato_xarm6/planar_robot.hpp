
#ifndef PLANAR_ROBOT_HPP
#define PLANAR_ROBOT_HPP
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
namespace tomato_xarm6
{
    class PlanarRobot
    {
    public:
        PlanarRobot(rclcpp::Node::SharedPtr node);
        ~PlanarRobot();

        void publish_planar_robot(double x, double y);
    private:
        rclcpp::Node::SharedPtr node_;
        rclcpp::Publisher<geometry_msgs::msg::Transform>::SharedPtr publisher_;
    };
}
#endif