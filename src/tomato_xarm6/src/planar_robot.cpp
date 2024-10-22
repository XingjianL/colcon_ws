#include "tomato_xarm6/planar_robot.hpp"
#include <random>
namespace tomato_xarm6
{
    PlanarRobot::PlanarRobot(rclcpp::Node::SharedPtr node) {
        node_ = node;
        publisher_ = node_->create_publisher<geometry_msgs::msg::Transform>("planar_robot_tf",10);
    }

    PlanarRobot::~PlanarRobot(){

    }

    void PlanarRobot::publish_planar_robot(double x, double y, double z){
        geometry_msgs::msg::Transform transform;

        // Set the translation
        transform.translation.x = x;
        transform.translation.y = y;
        transform.translation.z = z;

        // Set the rotation (quaternion)
        transform.rotation.x = -sqrt(2.)/2.;
        transform.rotation.y = 0.0;
        transform.rotation.z = -sqrt(2.)/2.;
        transform.rotation.w = 0.0;

        // other side
        // transform.rotation.x = 1.0;
        // transform.rotation.y = 0.0;
        // transform.rotation.z = 0.0;
        // transform.rotation.w = 0.0;
        
        publisher_->publish(transform);
    }

    
} // namespace tomato_xarm6
