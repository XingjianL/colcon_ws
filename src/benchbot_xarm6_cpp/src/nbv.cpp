#include "benchbot_xarm6_cpp/nbv.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <Eigen/Dense>
#include <vector>
#include "rclcpp_components/register_node_macro.hpp"

namespace benchbot_xarm6 {
    NBV::NBV(std::string &node_name) {
        node_ = std::make_shared<rclcpp::Node>(
            node_name,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        // nbv_sub_ = node_->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/predicted_pcd", 10, std::bind(&NBV::PredictedCallback, this, std::placeholders::_1)
        // );
        // nbv_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
        //     "/partial_pcd",10
        // );
        nbv_action_client = rclcpp_action::create_client<NBVAction>(
            node_,"nbv_server"
        );
    }
    NBV::~NBV() {
    }
    // void NBV::waiting_for_sync(){
    //     waiting_nbv_ = true;
    //     while(waiting_nbv_){
    //         RCLCPP_INFO(node_->get_logger(), "waiting for sync - NBV");
    //         std::this_thread::sleep_for(std::chrono::milliseconds(500));
    //         rclcpp::spin_some(node_);
    //     }
    //     RCLCPP_INFO(node_->get_logger(), "got NBV");
    // }

    // void NBV::PredictedCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg){
    //     waiting_nbv_ = false;
    //     RCLCPP_INFO(node_->get_logger(),"predicted callback %d",msg->is_dense);
    // }

    void NBV::publish_point_cloud(
        const std::shared_ptr<open3d::geometry::PointCloud>& pred_pc,
        const std::shared_ptr<open3d::geometry::PointCloud>& nbv_pc,
        PlantInfo& plant_info,
        std::tuple<uint8_t, uint8_t, uint8_t> segment_color,
        bool wait_for_nbv,
        std::string& pred_option) 
    {
        RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"), "plant_info: %p", static_cast<void*>(&plant_info));
        waiting_nbv_ = true;
        auto goal_msg = NBVAction::Goal();
        if (!nbv_action_client->wait_for_action_server()) {
            RCLCPP_ERROR(node_->get_logger(), "NBV Action server not available after waiting");
            rclcpp::shutdown();
        }
        std::string save_prefix = "output/pcd/plant_" + std::to_string(plant_info.creation_time.seconds()) + "/" +
            std::to_string(plant_info.id) + "/" + plant_info.plant_variant + "/" + std::to_string(std::get<0>(segment_color)) + "_" + 
            std::to_string(std::get<1>(segment_color)) + "_" + std::to_string(std::get<2>(segment_color));
        goal_msg.save_prefix = save_prefix;
        goal_msg.pred_option = pred_option;
        {
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"),"partial publish nbv input: %ld", nbv_pc->points_.size());
            goal_msg.nbv_input_pcd.width = nbv_pc->points_.size();
            goal_msg.nbv_input_pcd.is_dense = false;
            goal_msg.nbv_input_pcd.is_bigendian = false;
            sensor_msgs::PointCloud2Modifier modifier(goal_msg.nbv_input_pcd);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(nbv_pc->points_.size());
            sensor_msgs::PointCloud2Iterator<float> iter_x(goal_msg.nbv_input_pcd, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(goal_msg.nbv_input_pcd, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(goal_msg.nbv_input_pcd, "z");
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"),"adding data");
            for (size_t i = 0; i < nbv_pc->points_.size(); ++i, ++iter_x, ++iter_y, ++iter_z)//, ++iter_r, ++iter_g, ++iter_b)
            {
                const Eigen::Vector3d &point = pred_pc->points_[i];
                //const Eigen::Vector3d &color = o3d_pc->colors_[i];
                //
                *iter_x = point.x();
                *iter_y = point.y();
                *iter_z = point.z();
                // *iter_r = static_cast<uint8_t>(color.x() * 255.0);
                // *iter_g = static_cast<uint8_t>(color.y() * 255.0);
                // *iter_b = static_cast<uint8_t>(color.z() * 255.0);
            }
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"),"partial nbv input message created");
        }
        {
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"),"partial publish pred input: %ld", pred_pc->points_.size());
            goal_msg.pred_input_pcd.width = pred_pc->points_.size();
            goal_msg.pred_input_pcd.is_dense = false;
            goal_msg.pred_input_pcd.is_bigendian = false;
            sensor_msgs::PointCloud2Modifier modifier(goal_msg.pred_input_pcd);
            modifier.setPointCloud2FieldsByString(1, "xyz");
            modifier.resize(pred_pc->points_.size());
            sensor_msgs::PointCloud2Iterator<float> iter_x(goal_msg.pred_input_pcd, "x");
            sensor_msgs::PointCloud2Iterator<float> iter_y(goal_msg.pred_input_pcd, "y");
            sensor_msgs::PointCloud2Iterator<float> iter_z(goal_msg.pred_input_pcd, "z");
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"),"adding data");
            for (size_t i = 0; i < pred_pc->points_.size(); ++i, ++iter_x, ++iter_y, ++iter_z)//, ++iter_r, ++iter_g, ++iter_b)
            {
                const Eigen::Vector3d &point = pred_pc->points_[i];
                //const Eigen::Vector3d &color = o3d_pc->colors_[i];
                //
                *iter_x = point.x();
                *iter_y = point.y();
                *iter_z = point.z();
                // *iter_r = static_cast<uint8_t>(color.x() * 255.0);
                // *iter_g = static_cast<uint8_t>(color.y() * 255.0);
                // *iter_b = static_cast<uint8_t>(color.z() * 255.0);
            }
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"),"partial pred input message created");
        }
        
        auto send_goal_options = rclcpp_action::Client<NBVAction>::SendGoalOptions();
            send_goal_options.goal_response_callback =
              std::bind(&NBV::goal_response_callback, this, std::placeholders::_1);
            send_goal_options.feedback_callback =
              std::bind(&NBV::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
            send_goal_options.result_callback =
              std::bind(&NBV::result_callback, this, std::placeholders::_1, std::ref(plant_info), segment_color);
        
        auto future = nbv_action_client->async_send_goal(goal_msg, send_goal_options);
        rclcpp::spin_until_future_complete(node_, future);
        auto result = future.get();
        

        RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"), "Goal Confirmed %d, %d", result->get_status(), result->is_result_aware());
        if (wait_for_nbv) {
            if (!future.get()->is_result_aware()) {
                RCLCPP_WARN(node_->get_logger(), "Not Result Aware");
                return;
            }
            auto result_future = nbv_action_client->async_get_result(future.get());
            rclcpp::spin_until_future_complete(node_, result_future);            
        }
    }

    void NBV::goal_response_callback(const NBVGoalHandle::SharedPtr & goal_handle)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
        } else {
            RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"), "Goal accepted by server, waiting for result");
        }
    }

    void NBV::feedback_callback(
        NBVGoalHandle::SharedPtr,
        const std::shared_ptr<const NBVAction::Feedback> feedback)
    {
        RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"), "Feedback: %s", feedback->progress.c_str());
    }

    void NBV::result_callback(
        const NBVGoalHandle::WrappedResult & result,
        PlantInfo& plant_info,
        std::tuple<uint8_t, uint8_t, uint8_t> segment_color)
    {
        RCLCPP_INFO(node_->get_logger(), "Result Callback");
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(node_->get_logger(), "Goal Succeeded");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
                return;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
                return;
            default:
                RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
                return;
        }

        std::stringstream ss;
        ss << plant_info.plant_name << " " << plant_info.plant_variant << " "  
        << std::to_string(std::get<0>(segment_color)) << " "
        << std::to_string(std::get<1>(segment_color)) << " "
        << std::to_string(std::get<2>(segment_color)) << " "
        << " Result received: ";
        
        // for (auto number : result.result->optimal_order) {
        //     ss << number << " ";
        // }
        
        // plant_info.nbv_optimal_order = result.result->optimal_order;
        // plant_info.nbv_view_points = result.result->view_points;
        // plant_info.nbv_view_center = result.result->view_center;
        // plant_info.nbv_step += 1;

        for (auto & pc : plant_info.unique_point_clouds) {
            if (pc.segment_color == segment_color) {
                pc.nbv_optimal_order = result.result->optimal_order;
                pc.nbv_view_points = result.result->view_points;
                pc.nbv_view_center = result.result->view_center;
                pc.nbv_step += 1;
                pc.nbv_new_points = result.result->num_new_points;
                ss << std::to_string(result.result->num_new_points) << " | "
                << std::to_string(pc.o3d_pc->points_.size());
            }
        }
        RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
        // ss << plant_info.plant_name << " " 
        //     << plant_info.plant_variant << " "  
        //     << std::to_string(plant_info.instance_segmentation_id_b) 
        //     << " New NBV " << result.result->num_new_points << ": ";
        // for (auto number : result.result->optimal_order) {
        //     ss << number << " ";
        // }
        // ss << " view_points: ";
        // for (auto number : result.result->view_points) {
        //     ss << number << " ";
        // }
        // ss << " view_center: ";
        // for (auto number : result.result->view_center) {
        //     ss << number << " ";
        // }
        // ss << &plant_info;
        // RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
    }


}