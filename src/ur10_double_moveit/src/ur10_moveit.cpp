#include "ur10_double_moveit/ur10_moveit.hpp"
#include "tf2/LinearMath/Quaternion.h"

ur10_double::UR10MoveIt::UR10MoveIt(const std::string& position_csv, const std::string& orientation_csv)
{
    load_robot_setpoints(position_csv, orientation_csv);
}

ur10_double::UR10MoveIt::UR10MoveIt(const std::string& robot_name, const std::string& position_csv, const std::string& orientation_csv)
{
    robot_name_ = robot_name;
    load_robot_setpoints(position_csv, orientation_csv);
}
ur10_double::UR10MoveIt::UR10MoveIt(const std::string &robot_name, rclcpp::Node::SharedPtr node)
{
    robot_name_ = robot_name;
    using moveit::planning_interface::MoveGroupInterface;
    move_group_ = std::make_shared<MoveGroupInterface>(node, robot_name);
    configure_move_group();
}

ur10_double::UR10MoveIt::~UR10MoveIt()
{

}

void ur10_double::UR10MoveIt::load_robot_setpoints(const std::string& position_csv, const std::string& orientation_csv)
{
    robot_setpoints_ = ur10_double::read_robot_setpoints(position_csv, orientation_csv);
}

void ur10_double::UR10MoveIt::setpoint_control()
{
    auto next = next_setpoint();
    printf("setpoint: %f, %f, %f, %f, %f, %f\n", next.x, next.y, next.z, next.roll, next.pitch, next.yaw);
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
        // for (int i = 0; i < plan.trajectory_.joint_trajectory.points.size(); i++)
        // {
        //     auto const time = rclcpp::Duration::from_seconds(0.1 * i);
        //     plan.trajectory_.joint_trajectory.points.at(i).time_from_start.sec = time.seconds();
        //     plan.trajectory_.joint_trajectory.points.at(i).time_from_start.nanosec = time.nanoseconds();
        // }
        // printf("plan succeeded\n");
        // printf("plan, %u, %u, %f, %f, %f\n", 
        //     plan.trajectory.joint_trajectory.points.at(0).time_from_start.sec,
        //     plan.trajectory.joint_trajectory.points.at(0).time_from_start.nanosec
        //     , 
        //     plan.trajectory.joint_trajectory.points.at(0).positions.at(0),
        //     plan.trajectory.joint_trajectory.points.at(0).positions.at(1),
        //     plan.trajectory.joint_trajectory.points.at(0).positions.at(2));
        // printf("plan, %u, %u, %f, %f, %f\n", 
        //     plan.trajectory.joint_trajectory.points.at(1).time_from_start.sec,
        //     plan.trajectory.joint_trajectory.points.at(1).time_from_start.nanosec
        //     , 
        //     plan.trajectory.joint_trajectory.points.at(1).positions.at(0),
        //     plan.trajectory.joint_trajectory.points.at(1).positions.at(1),
        //     plan.trajectory.joint_trajectory.points.at(1).positions.at(2));
        // printf("plan, %u, %u, %f, %f, %f\n", 
        //     plan.trajectory.joint_trajectory.points.at(2).time_from_start.sec,
        //     plan.trajectory.joint_trajectory.points.at(2).time_from_start.nanosec
        //     , 
        //     plan.trajectory.joint_trajectory.points.at(2).positions.at(0),
        //     plan.trajectory.joint_trajectory.points.at(2).positions.at(1),
        //     plan.trajectory.joint_trajectory.points.at(2).positions.at(2));
        move_group_->execute(plan);
        setpoint_index_++;
    }
    else
    {
        printf("plan failed\n");
        RCLCPP_ERROR(rclcpp::get_logger("ur10_double_moveit"), "Planning failed!");
        move_group_->clearPoseTargets();
        move_group_->setJointValueTarget(move_group_->getNamedTargetValues("h"));
        auto const [success, plan] = [this]{
            moveit::planning_interface::MoveGroupInterface::Plan plan;
            auto const ok = static_cast<bool>(this->move_group_->plan(plan));
            return std::make_pair(ok, plan);
        }();
        if (success)
        {
            printf("go home\n");
            move_group_->execute(plan);
        }
    }
    move_group_->clearPoseTargets();
}

void ur10_double::UR10MoveIt::configure_move_group(){
    if (move_group_ == nullptr){
        RCLCPP_ERROR(rclcpp::get_logger("ur10_double_moveit"), "No move_group_ provided!");
        return;
    }
    //move_group_->setPlannerId("RRTstarkConfigDefault");
    move_group_->setPlanningTime(5.0);
    move_group_->setNumPlanningAttempts(3);
    move_group_->setMaxVelocityScalingFactor(1);
    move_group_->setMaxAccelerationScalingFactor(1);
}

ur10_double::setpoints ur10_double::UR10MoveIt::next_setpoint()
{
    if (setpoint_index_ + 1 < robot_setpoints_.size()){
        return robot_setpoints_[setpoint_index_ + 1];
    }
    RCLCPP_ERROR(rclcpp::get_logger("ur10_double_moveit"), "End of setpoints reached!");
    return ur10_double::setpoints{0.0, 0.0, 0.0, 1000.0, 10000.0, 100000.0};
}