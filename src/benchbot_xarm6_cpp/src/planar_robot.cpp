#include "benchbot_xarm6_cpp/planar_robot.hpp"
#include <random>
#include <Eigen/Dense>
namespace benchbot_xarm6
{
    PlanarRobot::PlanarRobot(rclcpp::Node::SharedPtr node, std::string robot_prefix, bool repub_jointstates) {
        node_ = node;
        jointstates_.position.clear();
        jointstates_.name.clear();
        jointstates_.position = {0};
        jointstates_.name = {"none"};
        robot_prefix_ = robot_prefix;
        planar_publisher_ = node_->create_publisher<geometry_msgs::msg::Transform>("ue5/"+robot_prefix+"/planar_robot_tf",10);
        joints_publisher_ = node_->create_publisher<sensor_msgs::msg::JointState>("ue5/"+robot_prefix+"/joint_states",10);
        repub_jointstates_ = repub_jointstates;
        if (repub_jointstates_) {
            joint_state_subscriber_ = node_->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states", 10, std::bind(&PlanarRobot::JointStateCallback, this, std::placeholders::_1));
        }
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlanarRobot::publish_both, this)
        );
    }

    PlanarRobot::~PlanarRobot(){
        timer_->cancel();
    }

    void PlanarRobot::publish_planar_robot(){
        planar_publisher_->publish(transform_);
    }
    void PlanarRobot::publish_joints_robot(){
        jointstates_.header.stamp = node_->get_clock()->now();
        joints_publisher_->publish(jointstates_);
    }
    void PlanarRobot::publish_both(){
        if (repub_jointstates_){
            publish_joints_robot();
        }
        if (!publish_topic_){
            
            return;
        }
        publish_planar_robot();
        publish_joints_robot();
    }
    void PlanarRobot::set_planar_targets(double x, double y, double z, double rand_rot){
        // Set the translation
        transform_.translation.x = x;
        transform_.translation.y = y;
        transform_.translation.z = z;

        Eigen::Quaterniond q1(1,0,0,0);
        Eigen::Quaterniond rand_q = Eigen::AngleAxisd(rand_rot, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(rand_rot, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(rand_rot, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond final_rot = rand_q * q1;

        // Set the rotation (quaternion)
        transform_.rotation.x = final_rot.x();
        transform_.rotation.y = final_rot.y();
        transform_.rotation.z = final_rot.z();
        transform_.rotation.w = final_rot.w();
        publish_topic_ = true;
    }
    void PlanarRobot::set_joints_targets(std::vector<std::string> name, std::vector<double> target){
        jointstates_.position = target;
        jointstates_.name = name;
        publish_topic_ = true;
    }

    void PlanarRobot::JointStateCallback(const sensor_msgs::msg::JointState::ConstSharedPtr& msg) {
        set_joints_targets(msg->name, msg->position);
        publish_topic_ = false;
    }

} // namespace benchbot_xarm6
