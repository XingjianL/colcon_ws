
#ifndef ENVIRONMENT_INFO_HPP
#define ENVIRONMENT_INFO_HPP
#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "benchbot_xarm6_cpp/unique_point_cloud.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"

namespace benchbot_xarm6
{
    struct PlantInfo
    {
        std::string transforms;
        uint32_t id;
        std::string plant_name;
        std::string plant_variant;
        uint8_t instance_segmentation_id;

        bool operator==(const PlantInfo& rhs) const;

        std::vector<UniquePointCloud> unique_point_clouds;
        void ParseData(const std::string& data);
    };
    struct RobotInfo
    {
        std::string name;
        std::string position;
        std::string orientation;
        std::string description;
        std::shared_ptr<benchbot_xarm6::ImageSubscriber> image_subscriber;

        bool operator==(const RobotInfo& rhs) const;
        
        void ConfigCamera(rclcpp::Node::SharedPtr node);
        void ParseData(const std::string& data);
    };

    class EnvironmentInfo
    {
    public:
        EnvironmentInfo(rclcpp::Node::SharedPtr node);
        ~EnvironmentInfo();

        std::vector<RobotInfo> robot_info_;
        std::vector<PlantInfo> plant_info_;

        void clear();
        void BuildPointClouds();
        void SavePointClouds();
    private:
        const std::string PLANTMARKER = "Plant";
        const std::string ROBOTMARKER = "Robot";
        rclcpp::Node::SharedPtr node_;
        void EnvStringCallback(const std_msgs::msg::String::ConstSharedPtr& msg);

        void ParseData(const std::string& data, std::vector<RobotInfo>& robot_info, std::vector<PlantInfo>& plant_info);
        std::vector<std::string> SplitByDelimiter(const std::string& data, const std::string& delimiter);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr environment_info_;        
    };
}
#endif