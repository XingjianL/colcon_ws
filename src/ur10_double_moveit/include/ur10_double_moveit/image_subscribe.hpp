
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


namespace ur10_double
{
    class ImageSubscriber
    {
    public:
        ImageSubscriber(rclcpp::Node::SharedPtr node);
        ~ImageSubscriber();

        void start();
        void stop();

    private:
        rclcpp::Node::SharedPtr node_;
        void RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_color_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_depth_;
        std::thread executor_thread_;
        rclcpp::executors::SingleThreadedExecutor executor_;

    };
}
#endif