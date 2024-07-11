#pragma once

#include <string>
#include "ur10_double_moveit/csv_parser.hpp"
#include <moveit/move_group_interface/move_group_interface.h>

namespace ur10_double {
    class UR10MoveIt
    {
    public:
        UR10MoveIt(const std::string& position_csv, const std::string& orientation_csv);
        UR10MoveIt(const std::string& robot_name, const std::string& position_csv, const std::string& orientation_csv);
        UR10MoveIt(const std::string& robot_name, rclcpp::Node::SharedPtr node);
        ~UR10MoveIt();

        void load_robot_setpoints(const std::string& position_csv, const std::string& orientation_csv);
        std::vector<ur10_double::setpoints> get_robot_setpoints() { return robot_setpoints_; };

        void setpoint_control();

        void configure_move_group();

    private:
        std::string robot_name_ = "ur10";
        std::vector<ur10_double::setpoints> robot_setpoints_;
        uint32_t setpoint_index_ = 0;

        std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
        ur10_double::setpoints next_setpoint();
    };
}