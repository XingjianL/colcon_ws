#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "benchbot_xarm6_cpp/xarm6_moveit.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"
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

  

  benchbot_xarm6::XARM6MoveIt robot1(
    "xarm6",
    node
  );
  robot1.load_robot_setpoints(
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_xyz.csv", 
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_rot.csv"
  );
  
  auto const image_subscriber = std::make_shared<benchbot_xarm6::ImageSubscriber>(node);
  image_subscriber->start();
  for (int i = 0; i < 40; i++){
    robot1.setpoint_control();
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    image_subscriber->publish_pcd_in_ros_ = true;
    while (image_subscriber->publish_pcd_in_ros_){
      
    }
    //rclcpp::sleep_for(std::chrono::milliseconds(1000));
  }
  while (!image_subscriber->image_queue_.empty()){
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(image_subscriber->image_queue_mutex_);
      if (!image_subscriber->image_queue_.empty()) {
        frame = image_subscriber->image_queue_.front();
        image_subscriber->image_queue_.pop();
      }
    }
    image_subscriber->video_writer_.write(frame);
  }
  robot1.load_robot_setpoints(
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_xyz.csv", 
    "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_rot.csv"
  );
  for (int i = 0; i < 40; i++){
    robot1.setpoint_control();
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    image_subscriber->publish_pcd_in_ros_ = true;
    while (image_subscriber->publish_pcd_in_ros_){
      
    }
  }
  while (!image_subscriber->image_queue_.empty()){
    cv::Mat frame;
    {
      std::lock_guard<std::mutex> lock(image_subscriber->image_queue_mutex_);
      if (!image_subscriber->image_queue_.empty()) {
        frame = image_subscriber->image_queue_.front();
        image_subscriber->image_queue_.pop();
      }
    }
    image_subscriber->video_writer_.write(frame);
  }
  image_subscriber->stop();

  rclcpp::shutdown();

  printf("goodbye world benchbot_xarm6_cpp package\n");

  return 0;
}
