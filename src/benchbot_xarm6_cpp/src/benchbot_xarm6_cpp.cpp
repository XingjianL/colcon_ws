#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "benchbot_xarm6_cpp/xarm6_moveit.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"
#include "benchbot_xarm6_cpp/environment_info.hpp"
#include "benchbot_xarm6_cpp/planar_robot.hpp"
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("benchbot_xarm6_cpp");

  printf("hello world benchbot_xarm6_cpp package\n");

  bool reconstruct_point_clouds = true;
  int sample_gap = 8;
  for (int i = 0; i < argc; i++)
  {
    std::string arg = argv[i];
    RCLCPP_INFO(logger, arg.c_str());
    if (arg.find("--no-pc") != std::string::npos) {
      printf("Skip Reconstructing PointClouds");
      reconstruct_point_clouds = false;
      sample_gap = 4;
    }
  }

  auto const env_node = std::make_shared<rclcpp::Node>(
    "benchbot_xarm6_cpp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  std::string nbv_node_name = "benchbot_nbv";
  std::shared_ptr<benchbot_xarm6::NBV> nbv = std::make_shared<benchbot_xarm6::NBV>(nbv_node_name);

  benchbot_xarm6::EnvironmentInfo env(env_node);            // UE5 environment parser
  benchbot_xarm6::XARM6MoveIt robot1("xarm6", env_node);    // MoveIt control
  benchbot_xarm6::PlanarRobot planar_platform(env_node);    // planar platform control
  std::string cam_node_name = "benchbot_xarm6_camera";
  env.waiting_for_sync();
  env.robot_info_[0].ConfigCamera(cam_node_name);
  env.robot_info_[0].image_subscriber->start();
  
  RCLCPP_INFO(logger, "Finished Init: Starting Setpoints");

  
  robot1.load_robot_setpoints(
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_xyz.csv", 
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_rot.csv"
  );
  for (int i = 0; i < 40; i+=sample_gap){
    robot1.setpoint_control(i);
    rclcpp::sleep_for(std::chrono::milliseconds(2500)); // wait for the robot in UE5 to settle
    for (int plant_id = 0; plant_id < 1; plant_id++){
      double platform_pos_x = (plant_id / 3) * 0.225 * 6;
      double platform_pos_y = (plant_id % 3) * 0.225 * 6;
      planar_platform.publish_planar_robot(
        platform_pos_x, platform_pos_y
      );
      env.EnvPublishCommand("Test:1");
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      env.waiting_for_sync();
      env.robot_info_[0].image_subscriber->waiting_for_sync();
      rclcpp::spin_some(env_node);
    
      if (reconstruct_point_clouds){
        env.BuildPointClouds(true);
        env.SavePointClouds();
        env.PredictPointCloud(nbv);
      } else {
        env.SaveRobotImages();
      }
      env.UpdateLog();
      
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  // write the rgb images received to video
  env.robot_info_[0].WriteVideo();

  robot1.load_robot_setpoints(
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_xyz.csv", 
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_rot.csv"
  );
  for (int i = 0; i < 40; i+=sample_gap){
    robot1.setpoint_control(i);
    rclcpp::sleep_for(std::chrono::milliseconds(2500));
    for (int plant_id = 0; plant_id < 1; plant_id++){
      double platform_pos_x = (plant_id / 3) * 0.225 * 6;
      double platform_pos_y = (plant_id % 3) * 0.225 * 6;
      planar_platform.publish_planar_robot(
        platform_pos_x, platform_pos_y
      );
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      env.waiting_for_sync();
      env.robot_info_[0].image_subscriber->waiting_for_sync();
      env.EnvPublishCommand("Test:1");
          // wait for the robot in UE5 to settle
      rclcpp::spin_some(env_node);
      if (reconstruct_point_clouds){
        env.BuildPointClouds(true);
        env.SavePointClouds();
        env.PredictPointCloud(nbv);
      } else {
        env.SaveRobotImages();
      }
      env.UpdateLog();
      rclcpp::sleep_for(std::chrono::milliseconds(100));
    }
  }

  env.robot_info_[0].WriteVideo();

  env.robot_info_[0].image_subscriber->stop();
  env.SaveLog();

  env.EnvPublishCommand("PCGSeedIncr:1");
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  rclcpp::shutdown();

  printf("goodbye world benchbot_xarm6_cpp package\n");

  return 0;
}
