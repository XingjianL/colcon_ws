#include "benchbot_xarm6_cpp/xarm6_moveit.hpp"
#include "tf2/LinearMath/Quaternion.h"
benchbot_xarm6::XARM6MoveIt::XARM6MoveIt(const std::string& position_csv, const std::string& orientation_csv)
{
    load_robot_setpoints(position_csv, orientation_csv);
}

benchbot_xarm6::XARM6MoveIt::XARM6MoveIt(const std::string& robot_name, const std::string& position_csv, const std::string& orientation_csv)
{
    robot_name_ = robot_name;
    load_robot_setpoints(position_csv, orientation_csv);
}
benchbot_xarm6::XARM6MoveIt::XARM6MoveIt(const std::string &robot_name, rclcpp::Node::SharedPtr node)
{
    robot_name_ = robot_name;
    using moveit::planning_interface::MoveGroupInterface;
    move_group_ = std::make_shared<MoveGroupInterface>(node, robot_name);
    configure_move_group();
    go_home();
}

benchbot_xarm6::XARM6MoveIt::~XARM6MoveIt()
{

}

void benchbot_xarm6::XARM6MoveIt::load_robot_setpoints(const std::string& position_csv, const std::string& orientation_csv)
{
    robot_setpoints_ = benchbot_xarm6::read_robot_setpoints(position_csv, orientation_csv);
    setpoint_index_ = 0;
}

void benchbot_xarm6::XARM6MoveIt::setpoint_control(int ind)
{
    auto next = next_setpoint();
    if (ind >= 0) {
        next = setpoint_ind(ind);
    }
    RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "loaded setpoint: %f, %f, %f, %f, %f, %f", next.x, next.y, next.z, next.roll, next.pitch, next.yaw);
    if (next.x == 0.0 && next.y == 0.0 && next.z == 0.0 && next.roll == 1000.0 && next.pitch == 10000.0 && next.yaw == 100000.0){
        return;
    }
    auto const target_pose = [&next](){
        tf2::Quaternion quat;
        quat.setRPY(next.roll, next.pitch, next.yaw);
        quat.normalize();
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation.w = quat.w();
        target_pose.orientation.x = quat.x();
        target_pose.orientation.y = quat.y();
        target_pose.orientation.z = quat.z();
        target_pose.position.x = next.x;
        target_pose.position.y = next.y;
        target_pose.position.z = next.z-2.5;
        return target_pose;
    }();
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);
    auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = static_cast<bool>(this->move_group_->plan(plan));
        return std::make_pair(ok, plan);
    }();

    if (success)
    {
        move_group_->execute(plan);
        setpoint_index_++;
    }
    else
    {
        RCLCPP_ERROR(rclcpp::get_logger("benchbot_xarm6_cpp"), "Planning failed! Planning to Home position...");
        go_home();
    }
    move_group_->clearPoseTargets();
    RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "Finished Movement");
}

void benchbot_xarm6::XARM6MoveIt::configure_move_group(){
    if (move_group_ == nullptr){
        RCLCPP_ERROR(rclcpp::get_logger("benchbot_xarm6_cpp"), "No move_group_ provided!");
        return;
    }
    //move_group_->setPlannerId("RRTConnectkConfigDefault");
    move_group_->setPlanningTime(1.0);
    move_group_->setNumPlanningAttempts(3);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
}

benchbot_xarm6::setpoints benchbot_xarm6::XARM6MoveIt::next_setpoint()
{
    if (setpoint_index_ + 1 < robot_setpoints_.size()){
        return robot_setpoints_[setpoint_index_ + 1];
    }
    RCLCPP_ERROR(rclcpp::get_logger("benchbot_xarm6_cpp"), "End of setpoints reached!");
    return benchbot_xarm6::setpoints{0.0, 0.0, 0.0, 1000.0, 10000.0, 100000.0};
}
benchbot_xarm6::setpoints benchbot_xarm6::XARM6MoveIt::setpoint_ind(uint32_t ind){
    if (ind >= robot_setpoints_.size()) {
        return next_setpoint();
    }
    return robot_setpoints_[ind];
}

void benchbot_xarm6::XARM6MoveIt::go_home()
{
    move_group_->setJointValueTarget(move_group_->getNamedTargetValues("home"));
    auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = static_cast<bool>(this->move_group_->plan(plan));
        return std::make_pair(ok, plan);
    }();
    if (success)
    {
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "Moving to Home position...");
        //printf("go home\n");
        move_group_->execute(plan);
    }
}

bool benchbot_xarm6::XARM6MoveIt::move_and_look_at(
    geometry_msgs::msg::Point move_goal, 
    geometry_msgs::msg::Point look_at_goal)
{
    geometry_msgs::msg::Pose target_pose;
    target_pose.position = move_goal;
    
    // Calculate the direction vector from the move_goal to the look_at_goal
    geometry_msgs::msg::Vector3 direction;
    direction.x = look_at_goal.x - move_goal.x;
    direction.y = look_at_goal.y - move_goal.y;
    direction.z = look_at_goal.z - move_goal.z;

    // Normalize the direction vector
    double length = std::sqrt(direction.x * direction.x + direction.y * direction.y + direction.z * direction.z);
    if (length > 0.0) {
        direction.x /= length;
        direction.y /= length;
        direction.z /= length;
    }

    // Create a quaternion to represent the orientation
    tf2::Quaternion quat;

    // Calculate the yaw (rotation around Z-axis) using atan2
    double yaw = std::atan2(direction.y, direction.x) - M_PI_2;
    //yaw = 0.0;
    // Calculate the pitch (rotation around Y-axis) using the arctangent of the vertical component
    double pitch =  - std::atan2(
                        direction.z, 
                        std::sqrt(direction.x * direction.x + direction.y * direction.y)
                    ) + M_PI_2;
    pitch = 0;
    // Set the roll (rotation around X-axis) to 0 because no rotation around X-axis is needed
    double roll =   std::atan2(
                        direction.z, 
                        std::sqrt(direction.x * direction.x + direction.y * direction.y)
                    ) - M_PI_2;
    //roll = 0;

    // Set the quaternion from roll, pitch, yaw
    quat.setRPY(roll, pitch, yaw);
    // Set the orientation of the target pose
    target_pose.orientation.w = quat.w();
    target_pose.orientation.x = quat.x();
    target_pose.orientation.y = quat.y();
    target_pose.orientation.z = quat.z();

    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();
    move_group_->setPoseTarget(target_pose);
    auto const [success, plan] = [this]{
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        auto const ok = static_cast<bool>(this->move_group_->plan(plan));
        return std::make_pair(ok, plan);
    }();
    if (success)
    {
        move_group_->execute(plan);
        move_group_->clearPoseTargets();
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "Finished LookAt Movement");
        return true;
    }
    else
    {
        move_group_->clearPoseTargets();
        return false;
    }
}
