
#ifndef ENVIRONMENT_INFO_HPP
#define ENVIRONMENT_INFO_HPP
#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <sstream>
#include <fstream>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/classification.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "tomato_xarm6/unique_point_cloud.hpp"
#include "tomato_xarm6/image_subscribe.hpp"

namespace tomato_xarm6
{
    void ParseUE5TransformString(std::string& transform, double (&result)[9]);

    struct PlantInfo
    {
        PlantInfo();

        rclcpp::Time creation_time;

        std::string transforms;
        uint32_t id;
        std::string plant_name;
        std::string plant_variant;
        uint8_t instance_segmentation_id_g;
        uint8_t instance_segmentation_id_b;
        int seedP;
        int seedL;
        std::string leaves;

        bool operator==(const PlantInfo& rhs) const;

        std::vector<UniquePointCloud> unique_point_clouds;
        void ParseData(const std::string& data);

        std::string csv_header;
        std::string csv_data;
        void UpdateLog();
    };
    struct RobotInfo
    {
        RobotInfo();

        rclcpp::Time creation_time;

        std::string name;
        std::string base_transforms;
        std::string camera_transforms;
        std::string camera_quaternion;
        
        double camera_FOV;
        int camera_width;
        int camera_height;


        bool should_update = false;
        void UpdateRobotStatus(const std::string &data);

        std::shared_ptr<tomato_xarm6::ImageSubscriber> image_subscriber;
        
        bool operator==(const RobotInfo& rhs) const;
        
        void ConfigCamera(std::string &node_name);
        void ParseData(const std::string& data);

        std::string csv_header;
        std::string csv_data;
        void UpdateLog();
        void WriteVideo();
    };

    class EnvironmentInfo
    {
    public:
        EnvironmentInfo(rclcpp::Node::SharedPtr node);
        ~EnvironmentInfo();

        void waiting_for_sync();
        bool waiting_msg = true;

        std::vector<RobotInfo> robot_info_;

        std::vector<PlantInfo> plant_info_;
        std::ofstream plant_log_file_;

        void clear();
        void BuildPointClouds(bool save_intermediate);
        void SavePointClouds();

        void UpdateLog();
        void SaveLog();

        void SaveRobotImages();        
        void EnvPublishCommand(const std::string& command);
        open3d::visualization::Visualizer visualizer;
    private:
        const std::string PLANTMARKER = "Tomato";
        const std::string ROBOTMARKER = "Robot";
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Node::SharedPtr node_;
        void EnvStringCallback(const std_msgs::msg::String::ConstSharedPtr& msg);

        void ParseData(const std::string& data, std::vector<RobotInfo>& robot_info, std::vector<PlantInfo>& plant_info);
        std::vector<std::string> SplitByDelimiter(const std::string& data, const std::string& delimiter);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr environment_info_;

        rclcpp::Time creation_time_;

        int pc_build_count_;  

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr env_publisher;
        
    };
}
#endif