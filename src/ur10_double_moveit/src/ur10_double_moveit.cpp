#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "ur10_double_moveit/ur10_moveit.hpp"
#include "ur10_double_moveit/image_subscribe.hpp"
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world ur10_double_moveit package\n");

  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "ur10_double_moveit",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("ur10_double_moveit");

  

  ur10_double::UR10MoveIt robot1(
    "robot1",
    node
  );
  robot1.load_robot_setpoints(
    "/home/xing2204/Lab/colcon_ws/src/ur10_double_moveit/setpoints/setpoints1_xyz.csv", 
    "/home/xing2204/Lab/colcon_ws/src/ur10_double_moveit/setpoints/setpoints1_rot.csv"
  );
  ur10_double::UR10MoveIt robot2(
    "robot2",
    node
  );
  robot2.load_robot_setpoints(
    "/home/xing2204/Lab/colcon_ws/src/ur10_double_moveit/setpoints/setpoints2_xyz.csv", 
    "/home/xing2204/Lab/colcon_ws/src/ur10_double_moveit/setpoints/setpoints2_rot.csv"
  );
  
  auto const image_subscriber = std::make_shared<ur10_double::ImageSubscriber>(node);
  image_subscriber->start();
  for (int i = 0; i < 10; i++){
    robot1.setpoint_control();
    robot2.setpoint_control();
  }
  
  image_subscriber->stop();

  rclcpp::shutdown();

  printf("goodbye world ur10_double_moveit package\n");

  return 0;
}
