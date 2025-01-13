#ifndef NBV_HPP
#define NBV_HPP
#pragma once
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include "custom_msg/action/pcd_nbv.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "open3d/Open3D.h"
namespace benchbot_xarm6
{
    class NBV
    {
        public:
            NBV(std::string &node_name);
            ~NBV();

            // void waiting_for_sync();
            bool waiting_nbv_ = true;
            void publish_point_cloud(const std::shared_ptr<open3d::geometry::PointCloud>& o3d_pc);
        private:
            rclcpp::Node::SharedPtr node_;
            // rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr nbv_sub_;
            // rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nbv_pub_;
            // void PredictedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg);

            using NBVAction = custom_msg::action::PcdNBV;
            using NBVGoalHandle = rclcpp_action::ClientGoalHandle<NBVAction>;

            rclcpp_action::Client<NBVAction>::SharedPtr nbv_action_client;
            void goal_response_callback(const NBVGoalHandle::SharedPtr & goal_handle);
            void feedback_callback(
                NBVGoalHandle::SharedPtr,
                const std::shared_ptr<const NBVAction::Feedback> feedback);
            void result_callback(const NBVGoalHandle::WrappedResult & result);
            // {
            //     RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->order);
            //     (void)uuid;
            //     return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
            // }


    };
}
#endif