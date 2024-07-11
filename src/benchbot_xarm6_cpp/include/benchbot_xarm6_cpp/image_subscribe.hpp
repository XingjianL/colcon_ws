
#ifndef IMAGE_SUBSCRIBE_HPP
#define IMAGE_SUBSCRIBE_HPP
#pragma once

#include <string>
#include <fstream>
#include <vector>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <rclcpp/rclcpp.hpp>
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace benchbot_xarm6
{
    class ImageSubscriber
    {
    public:
        ImageSubscriber(rclcpp::Node::SharedPtr node);
        ~ImageSubscriber();

        void start();
        void stop();

        void process_to_rgbd(rclcpp::Time callback_time);
        
        cv::VideoWriter video_writer_;
        std::queue<cv::Mat> image_queue_;
        std::mutex image_queue_mutex_;

        bool publish_pcd_in_ros_;
    private:
        rclcpp::Node::SharedPtr node_;
        void RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void SegmentImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_color_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_segment_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_depth_;

        void convert_to_ros_pointcloud(const open3d::geometry::PointCloud &pc, sensor_msgs::msg::PointCloud2 &msg, rclcpp::Time callback_time);
        sensor_msgs::msg::PointCloud2 ros_pc;
        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
        std::shared_ptr<open3d::geometry::PointCloud> o3d_pc;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

        void RGBDImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, const sensor_msgs::msg::Image::ConstSharedPtr& msg_segment, const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth);

        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_color_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_segment_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_depth_;
        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, sensor_msgs::msg::Image> ApproxTimeSyncPolicy;
        std::shared_ptr<message_filters::Synchronizer<ApproxTimeSyncPolicy>> sync_;

        std::thread executor_thread_;
        rclcpp::executors::SingleThreadedExecutor executor_;

        cv::Mat cv_img_;
        cv::Mat cv_img_segment_;
        cv::Mat cv_img_depth_;

        std::shared_ptr<open3d::visualization::Visualizer> vis_;
        std::thread vis_thread_;

        
    };
}
#endif