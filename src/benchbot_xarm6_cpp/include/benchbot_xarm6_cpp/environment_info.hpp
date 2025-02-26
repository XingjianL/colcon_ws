
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
#include <tf2_msgs/msg/tf_message.hpp>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

#include "benchbot_xarm6_cpp/unique_point_cloud.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"
#include "benchbot_xarm6_cpp/nbv.hpp"


namespace benchbot_xarm6
{
    class NBV; // nbv.hpp
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



        bool operator==(const PlantInfo& rhs) const;

        std::vector<UniquePointCloud> unique_point_clouds;
        std::shared_ptr<open3d::geometry::PointCloud> combined_point_cloud;
        
        void ParseData(const std::string& data);

        std::string csv_header;
        std::string csv_data;
        void UpdateLog();

        double CalcDistance(double base_t_x, double base_t_y, double base_t_z);
        int OptimalNBV();
        void CombinePointClouds();
    };
    struct RobotInfo
    {
        RobotInfo();

        rclcpp::Time creation_time;

        std::string name;
        std::string topic_name;
        double base_transforms[9];
        double camera_transforms[9];
        double camera_quaternions[4];
        
        double camera_FOV;
        int camera_width;
        int camera_height;

        rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr robot_transforms_subscriber_;
        void ConfigTFSubscriber(rclcpp::Node::SharedPtr node);
        void TFCallback(const tf2_msgs::msg::TFMessage::ConstSharedPtr& msg);
        bool should_update = false;
        void UpdateRobotStatus(const std::string &data);

        std::shared_ptr<benchbot_xarm6::ImageSubscriber> image_subscriber;
        
        bool operator==(const RobotInfo& rhs) const;
        
        void ConfigCamera(std::string &node_name, bool capture_both);
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
        void BuildPointClouds(bool save_intermediate, bool closest_plant);
        void SavePointClouds();
        void PredictPointCloud(const std::shared_ptr<NBV>& nbv_, int id, bool wait_for_nbv, std::string& pred_option);

        void UpdateLog();
        void SaveLog();

        void SaveRobotImages();        
        void EnvPublishCommand(const std::string& command);
        
        int GetClosestPlant(int robot_id);
        int GetRobotID(const std::string& robot_name);
        rclcpp::Time creation_time_;
        //open3d::visualization::Visualizer visualizer;
    private:
        const std::string PLANTMARKER = "Plant";
        const std::string ROBOTMARKER = "Robot";
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        rclcpp::Node::SharedPtr node_;
        void EnvStringCallback(const std_msgs::msg::String::ConstSharedPtr& msg);

        void ParseData(const std::string& data, std::vector<RobotInfo>& robot_info, std::vector<PlantInfo>& plant_info);
        std::vector<std::string> SplitByDelimiter(const std::string& data, const std::string& delimiter);
        rclcpp::Subscription<std_msgs::msg::String>::SharedPtr environment_info_;

        int pc_build_count_;  

        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr env_publisher;
        
    };
}
#endif