#include "ur10_double_moveit/image_subscribe.hpp"

namespace ur10_double {
    ImageSubscriber::ImageSubscriber(rclcpp::Node::SharedPtr node) 
    {
        node_ = node;
        image_sub_color_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/ue5/one/Color",
            10,
            std::bind(&ImageSubscriber::RGBImageCallback, this, std::placeholders::_1)
        );
        image_sub_depth_ = node->create_subscription<sensor_msgs::msg::Image>(
            "/ue5/one/Depth",
            10,
            std::bind(&ImageSubscriber::DepthImageCallback, this, std::placeholders::_1)
        );
    }

    ImageSubscriber::~ImageSubscriber() 
    {
        stop();
    }

    void ImageSubscriber::RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        std::cout << "imageCallback-RGB" << std::endl;
        try 
        {
            cv::Mat cv_img = cv_bridge::toCvShare(msg, "bgr8")->image;
            //cv::flip(cv_img, cv_img, -1);
            cv::imshow("color_view", cv_img);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
    void ImageSubscriber::DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        std::cout << "imageCallback-Depth" << std::endl;
        try 
        {
            cv::Mat cv_img = cv_bridge::toCvShare(msg, "32FC1")->image;

            cv::Mat normalized_image;
            cv::normalize(cv_img, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            //cv::flip(normalized_image, normalized_image, -1);
            cv::imshow("depth_view", normalized_image);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
        }
    }
    void ImageSubscriber::start()
    {
        executor_.add_node(node_);
        executor_thread_ = std::thread([this]() { executor_.spin(); });
    }

    void ImageSubscriber::stop()
    {
        executor_.cancel();
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
    }
}