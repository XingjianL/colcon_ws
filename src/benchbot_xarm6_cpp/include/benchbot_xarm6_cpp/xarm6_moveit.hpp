#pragma once

#include <string>
#include "benchbot_xarm6_cpp/csv_parser.hpp"
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
namespace benchbot_xarm6 {
    class XARM6MoveIt
    {
    public:
        XARM6MoveIt(const std::string& position_csv, const std::string& orientation_csv);
        XARM6MoveIt(const std::string& robot_name, const std::string& position_csv, const std::string& orientation_csv);
        XARM6MoveIt(const std::string& robot_name, rclcpp::Node::SharedPtr node);
        ~XARM6MoveIt();

        void load_robot_setpoints(const std::string& position_csv, const std::string& orientation_csv);
        std::vector<benchbot_xarm6::setpoints> get_robot_setpoints() { return robot_setpoints_; };

        void setpoint_control(int ind = -1);

        void configure_move_group();

        void go_home();

    private:
        std::string robot_name_ = "xarm6";
        std::vector<benchbot_xarm6::setpoints> robot_setpoints_;
        uint32_t setpoint_index_ = 0;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        benchbot_xarm6::setpoints next_setpoint();
        benchbot_xarm6::setpoints setpoint_ind(uint32_t ind);
    };
}