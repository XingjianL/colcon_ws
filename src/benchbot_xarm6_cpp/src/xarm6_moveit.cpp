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

void benchbot_xarm6::XARM6MoveIt::setpoint_control()
{
    auto next = next_setpoint();
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
        target_pose.position.z = next.z-0.5;
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
    move_group_->setPlannerId("RRTConnectkConfigDefault");
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