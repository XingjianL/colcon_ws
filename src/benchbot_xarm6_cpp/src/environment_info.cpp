#include "benchbot_xarm6_cpp/environment_info.hpp"
//#include "image_subscribe.hpp"
#include <ctime>
#include <chrono>
#include <filesystem>
#include <cmath>

namespace benchbot_xarm6 {

    /** 
     * transform format would be "Translation: X= Y= Z= Rotation: P= Y= R= Scale: X= Y= Z="
     * see https://docs.unrealengine.com/4.26/en-US/BlueprintAPI/Utilities/String/ToString_transform/
     * therefore result would be Xt,Yt,Zt,R,P,Y,Xs,Ys,Zs
     */
    void ParseUE5TransformString(std::string& transform, double (&result)[9]){
        

        std::vector<std::string> tokens;
        boost::split(tokens, transform, boost::is_any_of("="), boost::token_compress_off);
        if (tokens.size() < 9){
            RCLCPP_WARN(rclcpp::get_logger("benchbot_xarm_cpp"), "Incorrect Transform Message: %s", transform.c_str());
            return;
        }
        result[0] = -std::stod(tokens[1]); // negative because x-axis in UE is in opposite direction than ROS
        result[1] = std::stod(tokens[2]);
        result[2] = std::stod(tokens[3]);
        result[3] = std::stod(tokens[4]);
        result[4] = std::stod(tokens[5]);
        result[5] = std::stod(tokens[6]);
        result[6] = std::stod(tokens[7]);
        result[7] = std::stod(tokens[8]);
        result[8] = std::stod(tokens[9]);
    }

    // MARK: EnvironmentInfo
    EnvironmentInfo::EnvironmentInfo(rclcpp::Node::SharedPtr node) : node_(node) {
        creation_time_ = rclcpp::Clock(RCL_ROS_TIME).now();
        pc_build_count_ = 0;

        environment_info_ = node_->create_subscription<std_msgs::msg::String>(
            "/ue5/SceneInfo", 10, std::bind(&EnvironmentInfo::EnvStringCallback, this, std::placeholders::_1));
        //robot_info_.resize(1);
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        visualizer.CreateVisualizerWindow("RGBD Image", 640, 480);
        env_publisher = node_->create_publisher<std_msgs::msg::String>("/ue5/game_commands", 10);
    }

    EnvironmentInfo::~EnvironmentInfo() {}

    void EnvironmentInfo::waiting_for_sync(){
        waiting_msg = true;
        while(waiting_msg){
            RCLCPP_INFO(node_->get_logger(), "waiting for sync - Environment");
            EnvPublishCommand("GetSceneInfo:0");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
        RCLCPP_INFO(node_->get_logger(), "got Environment");
    }
    // MARK: env callback
    void EnvironmentInfo::EnvStringCallback(const std_msgs::msg::String::ConstSharedPtr& msg) {
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo");
        ParseData(msg->data, robot_info_, plant_info_);
        waiting_msg = false;
    }

    void EnvironmentInfo::ParseData(const std::string& data, std::vector<RobotInfo>& robot_info, std::vector<PlantInfo>& plant_info) {
        std::stringstream ss(data);
        std::vector<std::vector<std::string>> result;

        std::string line;
        while (std::getline(ss, line, '\n')) {
            result.push_back(SplitByDelimiter(line, "||"));
        }

        if (result.size() < 2) {
            RCLCPP_ERROR(rclcpp::get_logger("benchbot_xarm6_cpp"), "Incorrect environment data format on /ue5/SceneInfo! Should only have two lines.");
            return;
        }
        if (result[0].size() != result[1].size()){
            RCLCPP_ERROR(rclcpp::get_logger("benchbot_xarm6_cpp"), "Incorrect environment data format on /ue5/SceneInfo! Should have same number of elements in each line.");
            return;
        }

        for (size_t i = 0; i < result[0].size(); i++) {
            // check header aligns with PLANTMARKER ("Plant")
            if (result[0][i].substr(0, PLANTMARKER.length()) == PLANTMARKER) {
                PlantInfo plant;
                plant.ParseData(result[1][i]);
                plant.id = i;

                bool found = false;
                // check if plant with same properties exist
                for(auto &existing_plant : plant_info){
                    if(existing_plant == plant){
                        found = true;
                        break;
                    }
                }
                // add the plant to vector
                if(!found){
                    std::vector<std::string> tokens;
                    boost::split(tokens, result[1][i], boost::is_any_of(","), boost::token_compress_off);
                    RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), 
                        "adding new plant: %s, instance: %d, %d", 
                        result[0][i].c_str(), plant.instance_segmentation_id_g, plant.instance_segmentation_id_b);
                    plant.UpdateLog();
                    plant_info.push_back(plant);
                }
            // check header aligns with ROBOTMAKER ("Robot")
            } else if (result[0][i].substr(0, ROBOTMARKER.length()) == ROBOTMARKER) {
                RobotInfo robot;
                robot.ParseData(result[1][i]);
                bool found = false;
                for(auto &existing_robot : robot_info){
                    // check if robot with same properties exist
                    if(existing_robot == robot){
                        // need to update the existing robot for updated position
                        existing_robot.UpdateRobotStatus(result[1][i]);
                        found = true;
                        break;
                    }
                }
                // if not then add this robot to the vector
                if(!found){
                    RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "adding new robot: %s", result[0][i].c_str());
                    robot_info.push_back(robot);
                }
            }
        }
    }

    std::vector<std::string> EnvironmentInfo::SplitByDelimiter(const std::string& data, const std::string& delimiter) {
        std::vector<std::string> tokens;
        size_t pos = 0, start = 0;
        while ((pos = data.find(delimiter, start)) != std::string::npos) {
            tokens.push_back(data.substr(start, pos - start));
            start = pos + delimiter.length();
        }
        tokens.push_back(data.substr(start));
        return tokens;
    }

    void EnvironmentInfo::clear() {
        robot_info_.clear();
        plant_info_.clear();
    }

    void EnvironmentInfo::BuildPointClouds(bool save_intermediate) {
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::BuildPointClouds: Started");
        int robot_id = 0;
        for (size_t i = 0; i < robot_info_.size(); i++){
            if (robot_info_[i].topic_name == "xArm6"){
                robot_id = i;
            }
        }
        robot_info_[robot_id].image_subscriber->under_recon_ = true;
        pc_build_count_ += 1;

        // Parse UE5 robot base transformation to world
        double ue5_robot_base[9] = {0,0,0,0,0,0,0,0,0};
        ParseUE5TransformString(robot_info_[robot_id].base_transforms, ue5_robot_base);
        
        Eigen::Matrix4d transformation_mat_;
        transformation_mat_.setIdentity();

        // find transform between base (world) to camera
        auto frame_transform = tf_buffer_->lookupTransform(
            "world", "link_eef", rclcpp::Time(0)
        );

        const auto &translation = frame_transform.transform.translation;
        const auto &rotation = frame_transform.transform.rotation;
        Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
        q.normalize();
        Eigen::Matrix3d rot = q.toRotationMatrix();
        transformation_mat_.block<3, 3>(0, 0) = rot;
        transformation_mat_(0, 3) = translation.x - ue5_robot_base[0]/100.0;
        transformation_mat_(1, 3) = translation.y - ue5_robot_base[1]/100.0;
        transformation_mat_(2, 3) = translation.z;
        
        std::string save_intermediate_path = "";
        
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::BuildPointClouds: %f %f", ue5_robot_base[0]/100.0, ue5_robot_base[1]/100.0);
        for(auto& plant : plant_info_){
            if (save_intermediate) {
                save_intermediate_path = "output/pcd/plant_" + std::to_string(creation_time_.seconds()) + "/" +
                    std::to_string(plant.id) + "/" + plant.plant_variant;
                std::filesystem::path dir_path = save_intermediate_path;
                std::filesystem::create_directories(dir_path);
                save_intermediate_path = save_intermediate_path + "/" + std::to_string(pc_build_count_);
            }
            robot_info_[robot_id].image_subscriber->process_to_pc(
                plant.unique_point_clouds, 
                transformation_mat_,
                plant.instance_segmentation_id_g, plant.instance_segmentation_id_b,
                save_intermediate_path,
                visualizer);
        }
        
        robot_info_[robot_id].image_subscriber->under_recon_ = false;
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::BuildPointClouds: Finished");
        
    }

    void EnvironmentInfo::SavePointClouds() {
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::SavePointClouds: Started");
        for(auto& plant : plant_info_){
            for (auto& pc : plant.unique_point_clouds) {
                std::string filepath = "output/pcd/plant_" + std::to_string(creation_time_.seconds()) + "/" +
                    std::to_string(plant.id) + "/" + plant.plant_variant + "/";
                std::filesystem::path dir_path = filepath;
                std::filesystem::create_directories(dir_path);
                pc.savePointCloud(filepath);
            }
        }
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::SavePointClouds: Finished");
    }

    void EnvironmentInfo::PredictPointCloud(const std::shared_ptr<NBV>& nbv_) {
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::PredictPointCloud: Started");
        int closest_plant = 0;
        double closest_dist = 1e9;
        double base_transform[9] = {0,0,0,0,0,0,0,0,0};
        int robot_id = 0;
        for (size_t i = 0; i < robot_info_.size(); i++){
            if (robot_info_[i].topic_name == "xArm6"){
                robot_id = i;
            }
        }
        ParseUE5TransformString(robot_info_[robot_id].base_transforms, base_transform);
        
        for (size_t i = 0; i < plant_info_.size(); i++){
            auto dist = plant_info_[i].CalcDistance(base_transform[0],base_transform[1],base_transform[2]);
            if (dist < closest_dist) {
                closest_dist = dist;
                closest_plant = i;
            }
        }
        
        
        for(auto& pc : plant_info_[closest_plant].unique_point_clouds){
            RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::PredictPointCloud: Preparing Partial");
            auto pcd_copy = std::make_shared<open3d::geometry::PointCloud>(pc.o3d_pc->points_);
            Eigen::Vector3d translation_vector(
               base_transform[0]/100,
               base_transform[1]/100, 
               -1.1);
            // Eigen::Vector3d translation_vector(
            //     0,
            //     0, 
            //     -1.1);
            pcd_copy->Translate(translation_vector);
            std::string filepath = "output/pcd/";
            std::filesystem::path dir_path = filepath;
            std::filesystem::create_directories(dir_path);
            open3d::io::WritePointCloudOption o3d_option(true);
            std::string filename_ = std::to_string(pc_build_count_-1) + "temp.pcd";
            open3d::io::WritePointCloud(filepath + filename_, *pcd_copy, o3d_option);

            auto [filtered_pcd, inlier_ind] = pcd_copy->RemoveRadiusOutliers(16, 0.05);
            RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::PredictPointCloud: Translate %f %f %f", translation_vector.x(), translation_vector.y(), translation_vector.z());
            
            Eigen::Vector3d mean(0,0,0);
            for (const auto& point: filtered_pcd->points_){
                mean += point;
            }
            mean /= static_cast<double>(filtered_pcd->points_.size());
            RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::PredictPointCloud: mean %f %f %f", mean.x(), mean.y(), mean.z());

            if (filtered_pcd->points_.size() > 2000){
                filtered_pcd = filtered_pcd->FarthestPointDownSample(2000);
            }
            nbv_->publish_point_cloud(filtered_pcd);
        }
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo::PredictPointCloud: Finished");

    }

    void EnvironmentInfo::UpdateLog(){
        for(auto& robot : robot_info_){
            robot.UpdateLog();
        }
    }

    void EnvironmentInfo::SaveLog() {
        std::string file_name = std::to_string(creation_time_.seconds()) + ".csv";
        std::filesystem::path dir_path = "output/csv/";
        std::filesystem::create_directories(dir_path);
        std::ofstream robot_log_file_("output/csv/robot_" + file_name);
        bool header_added_ = false;
        for(auto& robot : robot_info_){
            if (!header_added_){
                header_added_ = true;
                robot_log_file_ << robot.csv_header;
            }
            robot_log_file_ << robot.csv_data;
        }
        robot_log_file_.close();

        std::ofstream plant_log_file_("output/csv/plant_" + file_name);
        header_added_ = false;
        for(auto& plant : plant_info_){
            if (!header_added_){
                header_added_ = true;
                plant_log_file_ << plant.csv_header;
            }
            plant_log_file_ << plant.csv_data;
        }
        plant_log_file_.close();
    }

    void EnvironmentInfo::SaveRobotImages()
    {
        for (size_t i = 0; i < robot_info_.size(); i++){
            if (robot_info_[i].topic_name == "xArm6"){
                robot_info_[i].image_subscriber->capture_count_ += 1;
                RCLCPP_INFO(rclcpp::get_logger("tomato_xarm6_camera"), "Saving images");
                robot_info_[i].image_subscriber->save_images("output/robot/images_"+std::to_string(creation_time_.seconds())+"/");
            }
        }
    }

    void EnvironmentInfo::EnvPublishCommand(const std::string& command)
    {
        auto message = std_msgs::msg::String();
        message.data = command;
        env_publisher->publish(message);
    }

    //MARK: PlantInfo

    PlantInfo::PlantInfo() {
        creation_time = rclcpp::Clock(RCL_ROS_TIME).now();
        csv_header = "Plant_Name,Plant_Variant,T_X,T_Y,T_Z,R,P,Y,S_X,S_Y,S_Z,ID_G,ID_B\n";
    }

    void PlantInfo::ParseData(const std::string &data) {
        std::vector<std::string> tokens;
        boost::split(tokens, data, boost::is_any_of(","), boost::token_compress_off);

        transforms = tokens[0];
        plant_name = tokens[1];
        instance_segmentation_id_g = static_cast<uint8_t>(std::stof(tokens[3])+0.1);//static_cast<uint8_t>( 255.0 * std::stof(tokens[3])+0.5);
        instance_segmentation_id_b = static_cast<uint8_t>(std::stof(tokens[4])+0.1);
        plant_variant = tokens[6];
    }

    bool PlantInfo::operator==(const PlantInfo &rhs) const {
        return id == rhs.id &&
            transforms == rhs.transforms && 
            plant_name == rhs.plant_name && 
            plant_variant == rhs.plant_variant && 
            instance_segmentation_id_g == rhs.instance_segmentation_id_g &&
            instance_segmentation_id_b == rhs.instance_segmentation_id_b;
    }

    void PlantInfo::UpdateLog() {
        double plant_transforms[9] = {0,0,0,0,0,0,0,0,0};
        ParseUE5TransformString(transforms, plant_transforms);
        csv_data += plant_name + ",";
        csv_data += plant_variant + ",";
        for (auto element : plant_transforms){
            csv_data += std::to_string(element);
            csv_data += ",";
        }
        csv_data += std::to_string(instance_segmentation_id_g);
        csv_data += ",";
        csv_data += std::to_string(instance_segmentation_id_b);
        csv_data += "\n";
    }

    double PlantInfo::CalcDistance(double base_t_x, double base_t_y, double base_t_z) {
        double plant_transforms[9] = {0,0,0,0,0,0,0,0,0};
        ParseUE5TransformString(transforms, plant_transforms);
        double dist = sqrt( pow(plant_transforms[0]-base_t_x,2) + 
                            pow(plant_transforms[1]-base_t_y,2) +
                            pow(plant_transforms[2]-base_t_z,2));
        return dist;
    }

    //MARK: RobotInfo
    RobotInfo::RobotInfo(){
        creation_time = rclcpp::Clock(RCL_ROS_TIME).now();
        csv_header = 
            "Robot_Name,Camera_FOV,ImageID,"
            "Robot_Base_T_X,Robot_Base_T_Y,Robot_Base_T_Z,"
            "Robot_Base_R,Robot_Base_P,Robot_Base_Y,"
            "Robot_Base_S_X,Robot_Base_S_Y,Robot_Base_S_Z,"
            "Camera_T_X,Camera_T_Y,Camera_T_Z,"
            "Camera_R,Camera_P,Camera_Y,"
            "Camera_S_X,Camera_S_Y,Camera_S_Z\n";
    }

    void RobotInfo::ParseData(const std::string &data) {
        std::vector<std::string> tokens;
        boost::split(tokens, data, boost::is_any_of(","), boost::token_compress_off);

        name = tokens[0]+tokens[1];
        topic_name = tokens[0];
        base_transforms = tokens[3];
        camera_FOV = std::stod(tokens[4]);
        camera_height = std::stoi(tokens[5]);
        camera_width = std::stoi(tokens[6]);
        camera_transforms = tokens[7];
        camera_quaternion = tokens[8];
    }

    void RobotInfo::ConfigCamera(std::string &node_name, bool capture_both) {
        image_subscriber = std::make_shared<ImageSubscriber>(node_name, camera_FOV, camera_width, camera_height, capture_both, topic_name);
        RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_camera"), "FOV: %f, width: %d, height: %d", camera_FOV, camera_width, camera_height);

        //image_subscriber->update_intrinsics(camera_FOV, camera_width, camera_height);
    }

    bool RobotInfo::operator==(const RobotInfo &rhs) const {
        
        return name == rhs.name;
    }

    void RobotInfo::UpdateRobotStatus(const std::string &data){
        ParseData(data);
        //RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "RobotInfo::UpdateRobotStatus: Finished");
    }

    void RobotInfo::UpdateLog(){
        csv_data += name + ",";
        csv_data += std::to_string(camera_FOV);
        csv_data += ",";
        if (image_subscriber != nullptr){
            csv_data += std::to_string(image_subscriber->capture_count_);
        } else {
            csv_data += "no camera";
        }
        csv_data += ",";
        double base_transform_arr[9] = {0,0,0,0,0,0,0,0,0};
        RCLCPP_INFO(rclcpp::get_logger("tomato_xarm6"), "%s", base_transforms.c_str());
        ParseUE5TransformString(base_transforms, base_transform_arr);
        for (auto element : base_transform_arr){
            csv_data += std::to_string(element);
            csv_data += ",";
        }
        double cam_transform_arr[9] = {0,0,0,0,0,0,0,0,0};
        RCLCPP_INFO(rclcpp::get_logger("tomato_xarm6"), "%s", camera_transforms.c_str());
        ParseUE5TransformString(camera_transforms, cam_transform_arr);
        for (auto element : cam_transform_arr){
            csv_data += std::to_string(element);
            csv_data += ",";
        }
        csv_data += "\n";
    }

    void RobotInfo::WriteVideo()
    {
        while (!image_subscriber->image_queue_.empty())
        {
            cv::Mat frame;
            {
                std::lock_guard<std::mutex> lock(image_subscriber->image_queue_mutex_);
    
                if (!image_subscriber->image_queue_.empty()) 
                {
                    frame = image_subscriber->image_queue_.front();
                    image_subscriber->image_queue_.pop();
                }
            }
            image_subscriber->video_writer_.write(frame);
        }
        
    }
}