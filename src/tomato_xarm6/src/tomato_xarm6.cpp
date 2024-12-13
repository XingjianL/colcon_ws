#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "tomato_xarm6/xarm6_moveit.hpp"
#include "tomato_xarm6/image_subscribe.hpp"
#include "tomato_xarm6/environment_info.hpp"
#include "tomato_xarm6/planar_robot.hpp"
#include <thread>
#include <random>

void spin_node_in_thread(rclcpp::Node::SharedPtr node)
{
    // Spin the node in a separate thread
    rclcpp::spin(node);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto const logger = rclcpp::get_logger("tomato_xarm6");

  printf("hello world tomato_xarm6 package\n");

  bool reconstruct_point_clouds = false;
  int sample_gap = 4;

  bool reset_time = false;
  std::string temperature = "3500,40,-4"; // temperature, intensity, exposure
  bool capture_both = false;
  unsigned int seed = 0;
  std::string filtered_disease = "";
  std::string split_height_leaf = "";
  std::string leaf_preprocess = "";
  for (int i = 1; i < argc; i++)
  {
    std::string arg = argv[i];
    RCLCPP_INFO(logger, arg.c_str());
    if (arg == "--reset-time") {
      reset_time = true;
    }
    if (arg == "--light-temp" && i + 1 < argc) {
      temperature = argv[i+1];
      ++i;
    }
    if (arg == "--both") {
      capture_both = true;
    }
    if (arg == "--seed" && i + 1 < argc) {
      seed = std::stoi(argv[i+1]);
      ++i;
    }
    if (arg == "--disease-filter" && i + 1 < argc){
      filtered_disease = argv[i+1];
      ++i;
    }
    if (arg == "--split-height-leaf" && i+1<argc){
      split_height_leaf = argv[i+1];
      ++i;
    }
    if (arg=="--preprocess" && i+1<argc){
      leaf_preprocess = argv[i+1];
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
  tomato_xarm6::PlanarRobot benchbot_platform(node, "BenchBot");
  tomato_xarm6::PlanarRobot spider_platform(node, "Spider");
  tomato_xarm6::PlanarRobot husky_platform(node, "Husky");
  std::thread spin_thread(spin_node_in_thread, node);

  std::string cam_node_name = "tomato_xarm6_camera";
  rclcpp::sleep_for(std::chrono::milliseconds(5000));
  if (reset_time) {
    env.EnvPublishCommand("TimeIncr:1:LightSet:"+temperature+":DiseaseFilter:"+filtered_disease+":SplitHeightLeaf:"+split_height_leaf+":LeafPreprocess:"+leaf_preprocess+":PCGSeedIncr:1");
  } else {
    env.EnvPublishCommand("TimeIncr:0:LightSet:"+temperature+":DiseaseFilter:"+filtered_disease+":SplitHeightLeaf:"+split_height_leaf+":LeafPreprocess:"+leaf_preprocess+":PCGSeedIncr:1");
  }
  rclcpp::sleep_for(std::chrono::milliseconds(10000));
  env.waiting_for_sync();
  for (size_t i = 0; i < env.robot_info_.size(); i++){
    RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "BenchBot");
    if (env.robot_info_[i].topic_name == "BenchBot"){
      env.robot_info_[i].ConfigCamera(cam_node_name, capture_both);
      env.robot_info_[i].image_subscriber->start();
    }
  }
  
  //env.waiting_for_sync();
  RCLCPP_INFO(logger, "Finished Init: Starting Setpoints");

  
  // robot1.load_robot_setpoints(
  //   "/home/lxianglabxing/colcon_ws/src/tomato_xarm6/setpoints/setpoints1_xyz.csv", 
  //   "/home/lxianglabxing/colcon_ws/src/tomato_xarm6/setpoints/setpoints1_rot.csv"
  // );
  for (int i = 0; i < 1; i+=sample_gap){
    //robot1.setpoint_control(i);
    rclcpp::sleep_for(std::chrono::milliseconds(1000)); // wait for the robot in UE5 to settle
    for (int plant_id_x = 1; plant_id_x < 10; plant_id_x++){ // 19
      double platform_pos_x = 1.0 * plant_id_x;//(plant_id / 3) * 0.225 * 6; 1.0 -> 19.0
      for (int plant_id_y = 1; plant_id_y < 2; plant_id_y++){ // 7
        double platform_pos_y = 1.0 * plant_id_y;//(plant_id % 3) * 0.225 * 6; 2.0 -2.0-> 12.0 (14.0 -2.0-> 4.0)
        
        // benchbot set position
        // set the base position (x, constant (y), constant (height), constant (rotation))
        benchbot_platform.set_planar_targets(
          platform_pos_x * 50, 525, 25, 0
        );
        // set the camera arm position over the horizontal bar (y, height(constant) )
        // this provides a more realistic visual, only using the planar also works
        benchbot_platform.set_joints_targets(
          {"benchbot_plate", "benchbot_camera"}, 
          {platform_pos_x * 50, platform_pos_y * 100}
        );
        RCLCPP_INFO(logger, "set benchbot pos");
        // spider set position (only planar, x,y,constant,constant)
        spider_platform.set_planar_targets(
          150 + platform_pos_x*10, 350, 0, 0
        );

        // husky set position
        RCLCPP_INFO(logger, "set spider pos");
        husky_platform.set_planar_targets(
          platform_pos_x * 100, 525, 0, 0
        );
        RCLCPP_INFO(logger, "set husky pos");
        
        rclcpp::sleep_for(std::chrono::milliseconds(1000));
        env.waiting_for_sync();
        for (size_t i = 0; i < env.robot_info_.size(); i++){
          RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "BenchBot");
          if (env.robot_info_[i].topic_name == "BenchBot"){
            env.robot_info_[i].image_subscriber->waiting_for_sync();
          }
        }
        if (reconstruct_point_clouds){
          env.BuildPointClouds(true);
          env.SavePointClouds();
          env.SaveRobotImages();
        } else {
          env.SaveRobotImages();
        }
        RCLCPP_INFO(logger, "Update Log");
        env.UpdateLog();
        RCLCPP_INFO(logger, "LogUpdated");
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
    }
  }
  
  //env.robot_info_[0].WriteVideo();
  RCLCPP_INFO(logger, "Saving Env File");
  env.SaveLog();
  RCLCPP_INFO(logger, "Stop Camera");
  for (size_t i = 0; i < env.robot_info_.size(); i++){
    RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[i].topic_name.c_str(), env.robot_info_[i].topic_name == "BenchBot");
    if (env.robot_info_[i].topic_name == "BenchBot"){
      env.robot_info_[i].ConfigCamera(cam_node_name, capture_both);
      env.robot_info_[i].image_subscriber->stop();
    }
  }

  //env.EnvPublishCommand("PCGSeedIncr:1");
  rclcpp::sleep_for(std::chrono::milliseconds(1000));
  //spin_thread.join();
  rclcpp::shutdown();
  printf("goodbye world tomato_xarm6 package\n");

  return 0;
}
