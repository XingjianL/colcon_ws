#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "benchbot_xarm6_cpp/xarm6_moveit.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"
#include "benchbot_xarm6_cpp/environment_info.hpp"
#include "benchbot_xarm6_cpp/planar_robot.hpp"

void spin_node_in_thread(rclcpp::Node::SharedPtr node)
{
    // Spin the node in a separate thread
    rclcpp::spin(node);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("benchbot_xarm6_cpp");

  printf("hello world benchbot_xarm6_cpp package\n");

  bool reconstruct_point_clouds = true;
  int sample_gap = 8;
  bool capture_both = false;
  // MARK: Arg List
  for (int i = 0; i < argc; i++)
  {
    std::string arg = argv[i];
    RCLCPP_INFO(logger, arg.c_str());
    if (arg.find("--no-pc") != std::string::npos) {
      printf("Skip Reconstructing PointClouds");
      reconstruct_point_clouds = false;
      sample_gap = 4;
    }
    if (arg == "--both") {
      capture_both = true;
    }
  }

  // MARK: Initializations
  auto const env_node = std::make_shared<rclcpp::Node>(
    "benchbot_xarm6_cpp",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
  );
  std::string nbv_node_name = "benchbot_nbv";
  std::shared_ptr<benchbot_xarm6::NBV> nbv = std::make_shared<benchbot_xarm6::NBV>(nbv_node_name);

  benchbot_xarm6::EnvironmentInfo env(env_node);            // UE5 environment parser
  benchbot_xarm6::XARM6MoveIt robot1("xarm6", env_node);    // MoveIt control
  benchbot_xarm6::PlanarRobot BenchBot_platform(env_node, "BenchBot", false);    // planar platform control
  benchbot_xarm6::PlanarRobot xArm_platform(env_node, "xArm6", true);    // planar platform control
  std::thread spin_thread(spin_node_in_thread, env_node);

  std::string cam_node_name = "benchbot_xarm6_camera";
  env.EnvPublishCommand("GetSceneInfo:0");
  env.waiting_for_sync();
  for (size_t i = 0; i < env.robot_info_.size(); i++){
    RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "xArm6");
    if (env.robot_info_[i].topic_name == "xArm6"){
      env.robot_info_[i].ConfigCamera(cam_node_name, capture_both);
      env.robot_info_[i].image_subscriber->start();
    }
  }
  env.EnvPublishCommand("GetSceneInfo:0");
  RCLCPP_INFO(logger, "Finished Init: Starting Setpoints");

  
  robot1.load_robot_setpoints(
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_xyz.csv", 
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_rot.csv"
  );
  for (int i = 0; i < 40; i+=sample_gap){
    robot1.setpoint_control(i);
    rclcpp::sleep_for(std::chrono::milliseconds(2500)); // wait for the robot in UE5 to settle
    for (int plant_id = 0; plant_id < 8; plant_id++){
      double platform_pos_x = plant_id;
      BenchBot_platform.set_planar_targets(
        50 - 124.848 + 50, 200, 25, 0
      );
      BenchBot_platform.set_joints_targets(
        {"benchbot_plate", "benchbot_camera"}, 
        {platform_pos_x * 50 + 16.785 + 100, 100.632-47}
      );
      rclcpp::sleep_for(std::chrono::milliseconds(1000));
      env.waiting_for_sync();
      for (size_t i = 0; i < env.robot_info_.size(); i++){
        RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "xArm6");
        if (env.robot_info_[i].topic_name == "xArm6"){
          env.robot_info_[i].image_subscriber->waiting_for_sync();
        }
      }
    
      if (reconstruct_point_clouds){
        env.BuildPointClouds(true);
        env.SavePointClouds();
        env.PredictPointCloud(nbv);
      } else {
        env.SaveRobotImages();
      }
      RCLCPP_INFO(logger, "Update Log");
      env.UpdateLog();
      RCLCPP_INFO(logger, "LogUpdated");
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      break;
    }
  }

  // write the rgb images received to video
  // env.robot_info_[0].WriteVideo();

  // robot1.load_robot_setpoints(
  //   "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_xyz.csv", 
  //   "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints2_rot.csv"
  // );
  // for (int i = 0; i < 40; i+=sample_gap){
  //   robot1.setpoint_control(i);
  //   rclcpp::sleep_for(std::chrono::milliseconds(2500)); // wait for the robot in UE5 to settle
  //   for (int plant_id = 0; plant_id < 1; plant_id++){
  //     double platform_pos_x = (plant_id / 3) * 0.225 * 6;
  //     double platform_pos_y = (plant_id % 3) * 0.225 * 6;
  //     BenchBot_platform.set_planar_targets(
  //       platform_pos_x * 50, 525, 25, 0
  //     );
  //     rclcpp::sleep_for(std::chrono::milliseconds(1000));
  //     env.waiting_for_sync();
  //     for (size_t i = 0; i < env.robot_info_.size(); i++){
  //       RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "BenchBot");
  //       if (env.robot_info_[i].topic_name == "xArm6"){
  //         env.robot_info_[i].image_subscriber->waiting_for_sync();
  //       }
  //     }
    
  //     if (reconstruct_point_clouds){
  //       env.BuildPointClouds(true);
  //       env.SavePointClouds();
  //       env.PredictPointCloud(nbv);
  //     } else {
  //       env.SaveRobotImages();
  //     }
  //     env.UpdateLog();
      
  //     rclcpp::sleep_for(std::chrono::milliseconds(100));
  //   }
  // }
  env.SaveLog();
  for (size_t i = 0; i < env.robot_info_.size(); i++){
    RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "BenchBot");
    if (env.robot_info_[i].topic_name == "xArm6"){
      env.robot_info_[i].ConfigCamera(cam_node_name, capture_both);
      env.robot_info_[i].image_subscriber->stop();
    }
  }
  

  //env.EnvPublishCommand("PCGSeedIncr:1");
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  rclcpp::shutdown();
  spin_thread.join();
  printf("goodbye world benchbot_xarm6_cpp package\n");

  return 0;
}
