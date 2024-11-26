#include "tomato_xarm6/planar_robot.hpp"
#include <random>
#include <Eigen/Dense>
namespace tomato_xarm6
{
    PlanarRobot::PlanarRobot(rclcpp::Node::SharedPtr node) {
        node_ = node;
        publisher_ = node_->create_publisher<geometry_msgs::msg::Transform>("planar_robot_tf",10);
    }

    PlanarRobot::~PlanarRobot(){

    }

    void PlanarRobot::publish_planar_robot(double x, double y, double z, double rand_rot){
        geometry_msgs::msg::Transform transform;

        // Set the translation
        transform.translation.x = x;
        transform.translation.y = y;
        transform.translation.z = z;

        Eigen::Quaterniond q1(0,-sqrt(2.)/2.,0,-sqrt(2.)/2.);
        Eigen::Quaterniond rand_q = Eigen::AngleAxisd(rand_rot, Eigen::Vector3d::UnitZ()) *
                                    Eigen::AngleAxisd(rand_rot, Eigen::Vector3d::UnitY()) *
                                    Eigen::AngleAxisd(rand_rot, Eigen::Vector3d::UnitX());
        Eigen::Quaterniond final_rot = rand_q * q1;

        // Set the rotation (quaternion)
        transform.rotation.x = final_rot.x();
        transform.rotation.y = final_rot.y();
        transform.rotation.z = final_rot.z();
        transform.rotation.w = final_rot.w();

        // other side
        // transform.rotation.x = 1.0;
        // transform.rotation.y = 0.0;
        // transform.rotation.z = 0.0;
        // transform.rotation.w = 0.0;
        
        publisher_->publish(transform);
    }

    
} // namespace tomato_xarm6
