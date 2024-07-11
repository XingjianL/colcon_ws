#pragma once

#include "ur10_double_moveit/ur10_moveit.hpp"
#include "ur10_double_moveit/csv_parser.hpp"
#include <rclcpp/rclcpp.hpp>

namespace ur10_double {
    class UR10
    {
    public:
        UR10(const std::string& position_csv, const std::string& orientation_csv);
        UR10(const std::string& robot_name, const std::string& position_csv, const std::string& orientation_csv);
        UR10(const std::string& robot_name, rclcpp::Node::SharedPtr node);
        ~UR10();
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