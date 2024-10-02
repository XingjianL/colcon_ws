#include "benchbot_xarm6_cpp/nbv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <Eigen/Dense>
#include <vector>
namespace benchbot_xarm6 {
    NBV::NBV(std::string &node_name) {
        node_ = std::make_shared<rclcpp::Node>(
            node_name,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        nbv_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/predicted_pcd", 10, std::bind(&NBV::PredictedCallback, this, std::placeholders::_1)
        );
        nbv_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
            "partial_pcd",10
        );
    }
    NBV::~NBV() {

    }
    void NBV::waiting_for_sync(){
        waiting_nbv_ = true;
        while(waiting_nbv_){
            RCLCPP_INFO(node_->get_logger(), "waiting for sync - NBV");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            rclcpp::spin_some(node_);
        }
        RCLCPP_INFO(node_->get_logger(), "got NBV");
    }

    void NBV::PredictedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
        waiting_nbv_ = false;
        RCLCPP_INFO(node_->get_logger(),"predicted callback %d",msg->is_dense);
    }

    void NBV::publish_point_cloud(const std::shared_ptr<open3d::geometry::PointCloud>& o3d_pc) {
        
        
        sensor_msgs::msg::PointCloud2 msg;
        msg.width = o3d_pc->points_.size();
        msg.is_dense = false;
        msg.is_bigendian = false;
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(o3d_pc->points_.size());
        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        RCLCPP_INFO(node_->get_logger(),"partial publish: %ld", o3d_pc->points_.size());
        for (size_t i = 0; i < o3d_pc->points_.size(); ++i, ++iter_x, ++iter_y, ++iter_z)//, ++iter_r, ++iter_g, ++iter_b)
        {
            const Eigen::Vector3d &point = o3d_pc->points_[i];
            //const Eigen::Vector3d &color = o3d_pc->colors_[i];
            //
            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();
            // *iter_r = static_cast<uint8_t>(color.x() * 255.0);
            // *iter_g = static_cast<uint8_t>(color.y() * 255.0);
            // *iter_b = static_cast<uint8_t>(color.z() * 255.0);
        }
        nbv_pub_->publish(msg);
    }
}