#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "benchbot_xarm6_cpp/xarm6_moveit.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"
#include "benchbot_xarm6_cpp/environment_info.hpp"
int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;

  printf("hello world benchbot_xarm6_cpp package\n");

  rclcpp::init(argc, argv);
  auto const node = std::make_shared<rclcpp::Node>(
    "benchbot_xarm6_cpp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  auto const logger = rclcpp::get_logger("benchbot_xarm6_cpp");

  
  benchbot_xarm6::EnvironmentInfo env(node);
  

  benchbot_xarm6::XARM6MoveIt robot1(
    "xarm6",
    node
  );
  robot1.load_robot_setpoints(
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_xyz.csv", 
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_rot.csv"
  );
  
  env.robot_info_[0].ConfigCamera(node);
  env.robot_info_[0].image_subscriber->start();
  // auto const image_subscriber = std::make_shared<benchbot_xarm6::ImageSubscriber>(node);
  // image_subscriber->start();
  for (int i = 0; i < 40; i+=7){
    robot1.setpoint_control();
    rclcpp::sleep_for(std::chrono::milliseconds(3000));
    env.BuildPointClouds();
    env.SavePointClouds();
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
  }
  while (!env.robot_info_[0].image_subscriber->image_queue_.empty()){
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(env.robot_info_[0].image_subscriber->image_queue_mutex_);
      if (!env.robot_info_[0].image_subscriber->image_queue_.empty()) {
        frame = env.robot_info_[0].image_subscriber->image_queue_.front();
        env.robot_info_[0].image_subscriber->image_queue_.pop();
      }
    }
    env.robot_info_[0].image_subscriber->video_writer_.write(frame);
  }
  robot1.load_robot_setpoints(
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_xyz.csv", 
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_rot.csv"
  );
  for (int i = 0; i < 40; i+=7){
    robot1.setpoint_control();
    rclcpp::sleep_for(std::chrono::milliseconds(3000));
    env.BuildPointClouds();
    env.SavePointClouds();
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
  }
  while (!env.robot_info_[0].image_subscriber->image_queue_.empty()){
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(env.robot_info_[0].image_subscriber->image_queue_mutex_);
      if (!env.robot_info_[0].image_subscriber->image_queue_.empty()) {
        frame = env.robot_info_[0].image_subscriber->image_queue_.front();
        env.robot_info_[0].image_subscriber->image_queue_.pop();
      }
    }
    env.robot_info_[0].image_subscriber->video_writer_.write(frame);
  }
  env.robot_info_[0].image_subscriber->stop();

  rclcpp::shutdown();

  printf("goodbye world benchbot_xarm6_cpp package\n");

  return 0;
}
