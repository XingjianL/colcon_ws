
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


#include "tomato_xarm6/unique_point_cloud.hpp"

namespace tomato_xarm6
{
    class ImageSubscriber
    {
    public:
        ImageSubscriber(std::string &node_name, double camera_FOV, int width, int height, bool capture_both, std::string &topic_name);
        ~ImageSubscriber();

        void start();
        void stop();

        void update_intrinsics(double fov, int width, int height);
        int capture_count_ = 0;
        bool process_to_pc( 
            std::vector<UniquePointCloud>& unique_pcs,
            const Eigen::Matrix4d &transform,
            uint8_t instance_id_g, 
            uint8_t instance_id_b,
            std::string& save_intermediate,
            open3d::visualization::Visualizer& visualizer
            );
        void save_images(std::string path);
        void clear_images();

        cv::VideoWriter video_writer_;
        std::queue<cv::Mat> image_queue_;
        std::mutex image_queue_mutex_;

        cv::Mat cv_img_;
        cv::Mat cv_img_segment_;
        cv::Mat cv_img_depth_;
        cv::Mat cv_img1_;
        cv::Mat cv_img1_depth_;

        void waiting_for_sync();
        bool under_recon_;
        bool capture_both_ = false;
        bool waiting_msg_rgbd = true;
        bool waiting_msg_stereo = true;
        std::mutex waiting_msg_mutex;
        
    private:
        cv::Mat depth_cmeters_;
        cv::Mat rgb_image_;

        std::set<std::tuple<uchar, uchar, uchar>> uniqueColors_;
        open3d::camera::PinholeCameraIntrinsic intrinsics_;

        rclcpp::Node::SharedPtr node_;
        void RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void SegmentImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);
        void DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg);

        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_color_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_segment_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_depth_;

        void convert_to_ros_pointcloud(const open3d::geometry::PointCloud &pc, sensor_msgs::msg::PointCloud2 &msg, rclcpp::Time callback_time);
        sensor_msgs::msg::PointCloud2 ros_pc;

        //std::shared_ptr<open3d::geometry::PointCloud> o3d_pc;
        //std::vector<tomato_xarm6::UniquePointCloud> o3d_pc_vector_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;

        void StereoImageCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color1, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth,
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth1);

        void RGBDImageCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_segment, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth);

        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_color_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_color1_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_segment_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_depth_;
        message_filters::Subscriber<sensor_msgs::msg::Image> sync_sub_depth1_;

        // change the template to number of sync_sub above
        typedef message_filters::sync_policies::ApproximateTime
        <   sensor_msgs::msg::Image, 
            sensor_msgs::msg::Image, 
            sensor_msgs::msg::Image,
            sensor_msgs::msg::Image> ApproxTimeSyncPolicyStereo;
        std::shared_ptr<message_filters::Synchronizer<ApproxTimeSyncPolicyStereo>> stereo_sync_;
        
        typedef message_filters::sync_policies::ApproximateTime
        <   sensor_msgs::msg::Image, 
            sensor_msgs::msg::Image, 
            sensor_msgs::msg::Image> ApproxTimeSyncPolicyRGBD;
        std::shared_ptr<message_filters::Synchronizer<ApproxTimeSyncPolicyRGBD>> rgbd_sync_;

        std::thread executor_thread_;
        rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    };
}
#endif