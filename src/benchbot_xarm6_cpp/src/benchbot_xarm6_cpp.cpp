#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include "benchbot_xarm6_cpp/xarm6_moveit.hpp"
#include "benchbot_xarm6_cpp/image_subscribe.hpp"
#include "benchbot_xarm6_cpp/environment_info.hpp"
#include "benchbot_xarm6_cpp/planar_robot.hpp"

#include <random>
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
  int plant_id_gap = 2;
  double arm_randomness = M_PI / 10;
  bool capture_both = false;
  bool pred = false;
  bool reset_time = false;
  bool closest = false;
  int nbv_color_id = -1;
  std::string PCGSeedIncr = "1";
  std::string temperature = "3500,0,1";
  // MARK: Arg List
  for (int arg_i = 0; arg_i < argc; arg_i++)
  {
    std::string arg = argv[arg_i];
    RCLCPP_INFO(logger, arg.c_str());
    if (arg.find("--no-pc") != std::string::npos) {
      printf("Skip Reconstructing PointClouds");
      reconstruct_point_clouds = false;
      sample_gap = 4;
    }
    if (arg == "--both") {
      capture_both = true;
    }
    if (arg == "--pred") {
      pred = true;
    }
    if (arg == "--reset-time") {
      reset_time = true;
    }
    if (arg == "--light-temp" && arg_i + 1 < argc) {
      temperature = argv[arg_i+1];
      ++arg_i;
    }
    if (arg == "--arm-sample-gap" && arg_i + 1 < argc) {
      sample_gap = std::stoi(argv[arg_i+1]);
      ++arg_i;
    }
    if (arg == "--bench-sample-gap" && arg_i + 1 < argc) {
      plant_id_gap = std::stoi(argv[arg_i+1]);
      ++arg_i;
    }
    if (arg == "--arm_random_locations" && arg_i + 1 < argc) {
      arm_randomness = std::stod(argv[arg_i+1]);
      ++arg_i;
    }
    if (arg == "--closest") {
      closest = true;
    }
    if (arg == "--nbv-color-id" && arg_i + 1 < argc) {
      nbv_color_id = std::stoi(argv[arg_i+1]);
      ++arg_i;
    }
    if (arg == "--pcg-seed-incr" && arg_i + 1 < argc) {
      PCGSeedIncr = argv[arg_i+1];
      ++arg_i;
    }
  }

  // MARK: Initializations

  // environment connection and robots
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
  rclcpp::sleep_for(std::chrono::milliseconds(5000));
  if (reset_time) {
    env.EnvPublishCommand("TimeIncr:1:LightSet:"+temperature+":PCGSeedIncr:"+PCGSeedIncr);
  } else {
    env.EnvPublishCommand("TimeIncr:0:LightSet:"+temperature+":PCGSeedIncr:"+PCGSeedIncr);
  }
  rclcpp::sleep_for(std::chrono::milliseconds(10000));
  // camera init
  std::string cam_node_name = "benchbot_xarm6_camera";
  env.EnvPublishCommand("GetSceneInfo:0");
  env.waiting_for_sync();
  for (size_t robot_id = 0; robot_id < env.robot_info_.size(); robot_id++){
    //RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[robot_id].topic_name.c_str(), env.robot_info_[robot_id].topic_name == "xArm6");
    if (env.robot_info_[robot_id].topic_name == "xArm6"){
      env.robot_info_[robot_id].ConfigCamera(cam_node_name, capture_both);
      env.robot_info_[robot_id].image_subscriber->start();
    }
  }

  env.EnvPublishCommand("GetSceneInfo:0");
  RCLCPP_INFO(logger, "Finished Init: Starting Setpoints");
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> rot_dis(-arm_randomness, arm_randomness);
  std::uniform_real_distribution<> pos_dis(0, 0.1);
  robot1.load_robot_setpoints(
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_xyz.csv", 
    "/home/lxianglabxing/colcon_ws/src/benchbot_xarm6_cpp/setpoints/setpoints1_rot.csv"
  );


  //for (int xarm_sample = 0; xarm_sample < 5 * 16; xarm_sample+=sample_gap){
  for (int plant_id = 0; plant_id < 3*7; plant_id+=plant_id_gap){
    geometry_msgs::msg::Point move_goal;
    geometry_msgs::msg::Point look_at_goal;

    // move benchbot
    double platform_pos_x = plant_id % 7;
    double platform_pos_y = std::floor(plant_id / 7);
    BenchBot_platform.set_planar_targets(
      50 - 124.848 + 60 * platform_pos_y, 200, 25, 0
    );
    BenchBot_platform.set_joints_targets(
      {"benchbot_plate", "benchbot_camera"}, 
      {platform_pos_x * 60 + 16.785, 100.632-47}
    );

    look_at_goal.x = 0;
    look_at_goal.y = 0;
    look_at_goal.z = -0.9;
    move_goal.x = 0;
    move_goal.y = 0;
    move_goal.z = -0.5;
    // if (!pred && i > 0){
    //   float r = 0.4;
    //   float phi = std::clamp((M_PI / 2 / 5) * (i%5) + dis(gen), 0.0, M_PI_2);
    //   float theta = (2 * M_PI / 10) * (i/10) + dis(gen);
    //   move_goal.x = r * sin(phi) * cos(theta);
    //   move_goal.y = r * sin(phi) * sin(theta);
    //   move_goal.z = r * cos(phi)-0.9;
    // }
    // bool success = false;
    // int nbv_select = 0;
    robot1.move_and_look_at(move_goal, look_at_goal);

    rclcpp::sleep_for(std::chrono::milliseconds(2000)); // wait for the robot in UE5 to settle
    //for (int plant_id = 0; plant_id < 6*7; plant_id++){
    for (int xarm_sample = 0; xarm_sample < 5 * 16; xarm_sample+=sample_gap){

      RCLCPP_INFO(logger, "Running for %d, %d", xarm_sample, plant_id);
      // move benchbot
      // double platform_pos_x = plant_id % 7;
      // double platform_pos_y = std::floor(plant_id / 7);
      // BenchBot_platform.set_planar_targets(
      //   50 - 124.848 + 60 * platform_pos_y, 200, 25, 0
      // );
      // BenchBot_platform.set_joints_targets(
      //   {"benchbot_plate", "benchbot_camera"}, 
      //   {platform_pos_x * 60 + 16.785, 100.632-47-40}
      // );
      
      // move xArm within random point if not in prediction
      bool success = false;
      while (!pred && xarm_sample > 0 && !success){
        // without --pred flag
        {
          // generate xArm movement
          float r = 0.4;
          float phi = std::clamp(((M_PI_2*8/10+M_PI_2/10) / 5) * (xarm_sample%5) + rot_dis(gen), M_PI_2/10, M_PI_2*9/10);
          float theta = std::clamp((2 * M_PI / 16) * (xarm_sample/16) + rot_dis(gen), 0.0, 2*M_PI);
          move_goal.x = r * sin(phi) * cos(theta);
          move_goal.y = r * sin(phi) * sin(theta);
          move_goal.z = r * cos(phi)-0.9;
          look_at_goal.x = 0 + pos_dis(gen)-0.05;
          look_at_goal.y = 0 + pos_dis(gen)-0.05;
          look_at_goal.z = -0.9 + pos_dis(gen);
        }
        
        BenchBot_platform.set_joints_targets(
          {"benchbot_plate", "benchbot_camera"}, 
          {platform_pos_x * 60 + 16.785, 100.632-47-40}
        );
        
        success = robot1.move_and_look_at(move_goal, look_at_goal);
        if (success) {
          break;
        }
      }
      // sample for NBV
      if (pred){
        success = false;
        int nbv_view_select = 0;
        // get closest plant
        RCLCPP_INFO(logger, "pred - closest plant");
        int closest_plant_id = env.GetClosestPlant(env.GetRobotID("xArm6"));
        RCLCPP_INFO(logger, "pred - optimal nbv");
        int nbv_optimal_pipeline_ind = env.plant_info_[closest_plant_id].OptimalNBV();
        if (nbv_optimal_pipeline_ind < 0) {
          RCLCPP_INFO(logger, "pred - no optimal nbv");
        } else {
          BenchBot_platform.set_joints_targets(
            {"benchbot_plate", "benchbot_camera"}, 
            {platform_pos_x * 60 + 16.785, 100.632-47-40}
          );
          auto& optimal_pipeline = env.plant_info_[closest_plant_id].unique_point_clouds[nbv_optimal_pipeline_ind];
          for (int nbv_order : optimal_pipeline.nbv_optimal_order){
            RCLCPP_INFO(logger, "move and look at");
            {
              move_goal.x = optimal_pipeline.nbv_view_points[nbv_order * 3];
              move_goal.y = optimal_pipeline.nbv_view_points[nbv_order * 3 + 1];
              move_goal.z = optimal_pipeline.nbv_view_points[nbv_order * 3 + 2] - 0.9;
              look_at_goal.x = optimal_pipeline.nbv_view_center[0];
              look_at_goal.y = optimal_pipeline.nbv_view_center[1];
              look_at_goal.z = optimal_pipeline.nbv_view_center[2] - 0.9;
            }

            success = robot1.move_and_look_at(move_goal, look_at_goal);
            if(success){
              RCLCPP_INFO(logger, "moved and look at: to %d with priority %d", nbv_order, nbv_view_select);
              break;  // successful
            }
          nbv_view_select += 1;
          }
          if (!success){
            // failed: either no optimal order (not sampled yet) or all sampled points failed
            RCLCPP_INFO(logger, "No success, %d", optimal_pipeline.nbv_step);
          }
        }
      }
            

      rclcpp::sleep_for(std::chrono::milliseconds(2000));
      env.waiting_for_sync();
      
      // // sample for image
      // for (size_t robot_id = 0; robot_id < env.robot_info_.size(); robot_id++){
      //   //RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[robot_id].topic_name.c_str(), env.robot_info_[robot_id].topic_name == "xArm6");
      //   if (env.robot_info_[robot_id].topic_name == "xArm6"){
      //     env.robot_info_[robot_id].image_subscriber->waiting_for_sync();
      //   }
      // }
      // sample for image
      for (size_t robot_id = 0; robot_id < env.robot_info_.size(); robot_id++){
        //RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[robot_id].topic_name.c_str(), env.robot_info_[robot_id].topic_name == "xArm6");
        if (env.robot_info_[robot_id].topic_name == "xArm6"){
          env.robot_info_[robot_id].image_subscriber->waiting_for_sync();
        }
      }
      for (size_t robot_id = 0; robot_id < env.robot_info_.size(); robot_id++){
        //RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[robot_id].topic_name.c_str(), env.robot_info_[robot_id].topic_name == "xArm6");
        if (env.robot_info_[robot_id].topic_name == "xArm6"){
          env.robot_info_[robot_id].image_subscriber->waiting_for_sync();
        }
      }
      
      // save image, build point clouds, and estimate next-best-view
      if (reconstruct_point_clouds){
        //env.SaveRobotImages();
        env.BuildPointClouds(!closest, closest);
        env.SavePointClouds();
        if (pred) {
          env.PredictPointCloud(nbv, nbv_color_id, true);
          // while(nbv->waiting_nbv_) {
          //   //rclcpp::sleep_for(std::chrono::milliseconds(200));
          //   RCLCPP_INFO(logger, "Waiting NBV");
          // }
        }
      } else {
        env.SaveRobotImages();
      }
      RCLCPP_INFO(logger, "Update Log");
      env.UpdateLog();
      RCLCPP_INFO(logger, "LogUpdated");
      env.SaveLog();
      rclcpp::sleep_for(std::chrono::milliseconds(100));
      //break;
    }
  }

  env.SaveLog();
  //env.EnvPublishCommand("PCGSeedIncr:1");
  for (size_t robot_id = 0; robot_id < env.robot_info_.size(); robot_id++){
    //RCLCPP_INFO(logger, "robot names %s, %d", env.robot_info_[robot_id].topic_name.c_str(), env.robot_info_[robot_id].topic_name == "BenchBot");
    if (env.robot_info_[robot_id].topic_name == "xArm6"){
      env.robot_info_[robot_id].ConfigCamera(cam_node_name, capture_both);
      env.robot_info_[robot_id].image_subscriber->stop();
    }
  }
  

  //env.EnvPublishCommand("PCGSeedIncr:1");
  rclcpp::sleep_for(std::chrono::milliseconds(2000));
  rclcpp::shutdown();
  spin_thread.join();
  printf("goodbye world benchbot_xarm6_cpp package\n");

  return 0;
}
