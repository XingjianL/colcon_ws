#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tomato_xarm6/xarm6_moveit.hpp"
#include "tomato_xarm6/image_subscribe.hpp"
#include "tomato_xarm6/environment_info.hpp"
#include "tomato_xarm6/planar_robot.hpp"

#include <random>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("tomato_xarm6");

  printf("hello world tomato_xarm6 package\n");

  bool reconstruct_point_clouds = false;
  int sample_gap = 4;

  bool reset_time = false;
  int temperature = 3500;
  bool capture_both = false;
  unsigned int seed = 0;
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    RCLCPP_INFO(logger, arg.c_str());
    if (arg == "--reset-time") {
      reset_time = true;
    }
    if (arg == "--light-temp" && i + 1 < argc) {
      temperature = std::stoi(argv[i+1]);
      ++i;
    }
    if (arg == "--both") {
      capture_both = true;
    }
    if (arg == "--seed" && i + 1 < argc) {
      seed = std::stoi(argv[i+1]);
      ++i;
    }
  }
  std::mt19937 randgen(seed);
  std::uniform_real_distribution<double> distribution(-0.05, 0.05);

  RCLCPP_INFO(logger, "Received arguments %d",argc);
  auto const node = std::make_shared<rclcpp::Node>(
    "tomato_xarm6",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );

  tomato_xarm6::EnvironmentInfo env(node);            // UE5 environment parser
  //tomato_xarm6::XARM6MoveIt robot1("xarm6", node);    // MoveIt control
  tomato_xarm6::PlanarRobot planar_platform(node);    // planar platform control
  std::string cam_node_name = "tomato_xarm6_camera";
  env.waiting_for_sync();
  env.robot_info_[0].ConfigCamera(cam_node_name, capture_both);
  env.robot_info_[0].image_subscriber->start();
  if (reset_time) {
    env.EnvPublishCommand("TimeIncr:1:LightSet:"+std::to_string(temperature));
  } else {
    env.EnvPublishCommand("TimeIncr:0:LightSet:"+std::to_string(temperature));
  }
  RCLCPP_INFO(logger, "Finished Init: Starting Setpoints");

  
  // robot1.load_robot_setpoints(
  //   "/home/lxianglabxing/colcon_ws/src/tomato_xarm6/setpoints/setpoints1_xyz.csv", 
  //   "/home/lxianglabxing/colcon_ws/src/tomato_xarm6/setpoints/setpoints1_rot.csv"
  // );
  for (int i = 0; i < 1; i+=sample_gap){
    //robot1.setpoint_control(i);
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // wait for the robot in UE5 to settle
    for (int plant_id_x = 1; plant_id_x < 19; plant_id_x++){
      double platform_pos_x = 0.5 * plant_id_x;//(plant_id / 3) * 0.225 * 6; 1.0 -> 19.0
      for (int plant_id_y = 1; plant_id_y < 7; plant_id_y++){
        double platform_pos_y = 2.0 * plant_id_y;//(plant_id % 3) * 0.225 * 6; 2.0 -2.0-> 12.0 (14.0 -2.0-> 4.0)
        planar_platform.publish_planar_robot(
          platform_pos_x + distribution(randgen), platform_pos_y + distribution(randgen), 1.00 + distribution(randgen)
        );
        //env.robot_info_[0].image_subscriber->waiting_for_sync();
        //rclcpp::sleep_for(std::chrono::milliseconds(3500));
        env.waiting_for_sync();
        env.robot_info_[0].image_subscriber->waiting_for_sync();
        rclcpp::spin_some(node);
        env.robot_info_[0].image_subscriber->waiting_for_sync();
        if (reconstruct_point_clouds){
          env.BuildPointClouds(true);
          env.SavePointClouds();
          env.SaveRobotImages();
        } else {
          env.SaveRobotImages();
        }
        env.UpdateLog();
        
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
      env.EnvPublishCommand("Test:1");
    }
  }

  env.robot_info_[0].WriteVideo();

  env.robot_info_[0].image_subscriber->stop();
  env.SaveLog();

  env.EnvPublishCommand("PCGSeedIncr:1");
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  rclcpp::shutdown();

  printf("goodbye world tomato_xarm6 package\n");

  return 0;
}
