
#ifndef CSV_PARSER_HPP
#define CSV_PARSER_HPP
#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <rclcpp/rclcpp.hpp>

namespace ur10_double
{
    std::vector<double> csv2vector(const std::string& csv);
    struct setpoints{
        double x;
        double y;
        double z;
        double roll;
        double pitch;
        double yaw;
    };
    std::vector<setpoints> read_robot_setpoints(const std::string& position_csv, const std::string& orientation_csv);
}
#endif