#include "benchbot_xarm6_cpp/environment_info.hpp"
//#include "image_subscribe.hpp"

namespace benchbot_xarm6 {
    // MARK: EnvironmentInfo
    EnvironmentInfo::EnvironmentInfo(rclcpp::Node::SharedPtr node) : node_(node) {
        environment_info_ = node_->create_subscription<std_msgs::msg::String>(
            "/ue5/SceneInfo", 10, std::bind(&EnvironmentInfo::EnvStringCallback, this, std::placeholders::_1));
        robot_info_.resize(1);
    }

    EnvironmentInfo::~EnvironmentInfo() {}

    void EnvironmentInfo::EnvStringCallback(const std_msgs::msg::String::ConstSharedPtr& msg) {
        //RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "EnvironmentInfo");
        ParseData(msg->data, robot_info_, plant_info_);
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
            if (result[0][i].substr(0, PLANTMARKER.length()) == PLANTMARKER) {
                PlantInfo plant;
                plant.ParseData(result[1][i]);
                plant.id = i;

                bool found = false;
                for(auto &existing_plant : plant_info){
                    if(existing_plant == plant){
                        found = true;
                        break;
                    }
                }
                if(!found){
                    std::vector<std::string> tokens;
                    boost::split(tokens, result[1][i], boost::is_any_of(","), boost::token_compress_off);
                    RCLCPP_INFO(rclcpp::get_logger("benchbot_xarm6_cpp"), "adding new plant: %s, instance: %d", result[0][i].c_str(), plant.instance_segmentation_id);
                    
                    plant_info.push_back(plant);
                }
            } else if (result[0][i].substr(0, ROBOTMARKER.length()) == ROBOTMARKER) {
                RobotInfo robot;
                robot.ParseData(result[1][i]);
                bool found = false;
                for(auto &existing_robot : robot_info){
                    if(existing_robot == robot){
                        found = true;
                        break;
                    }
                }
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

    void EnvironmentInfo::BuildPointClouds() {
        robot_info_[0].image_subscriber->under_recon_ = true;
        for(auto& plant : plant_info_){
            robot_info_[0].image_subscriber->process_to_pc(plant.unique_point_clouds, plant.instance_segmentation_id);
        }
        robot_info_[0].image_subscriber->under_recon_ = false;
    }

    void EnvironmentInfo::SavePointClouds() {
        for(auto& plant : plant_info_){
            for (auto& pc : plant.unique_point_clouds) {
                std::string filepath = "/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/pcd/Plant" + 
                    std::to_string(plant.id) + "/" + plant.plant_variant + "/";
                pc.savePointCloud(filepath);
            }
        }
    }

    //MARK: PlantInfo
    void PlantInfo::ParseData(const std::string &data) {
        std::vector<std::string> tokens;
        boost::split(tokens, data, boost::is_any_of(","), boost::token_compress_off);

        this->transforms = tokens[0];
        this->plant_name = tokens[1];
        this->instance_segmentation_id = static_cast<uint8_t>(std::stof(tokens[3])+0.1);//static_cast<uint8_t>( 255.0 * std::stof(tokens[3])+0.5);
        this->plant_variant = tokens[5];
    }

    bool PlantInfo::operator==(const PlantInfo &rhs) const {
        return id == rhs.id &&
            transforms == rhs.transforms && 
            plant_name == rhs.plant_name && 
            plant_variant == rhs.plant_variant && 
            instance_segmentation_id == rhs.instance_segmentation_id;
    }

    //MARK: RobotInfo
    void RobotInfo::ParseData(const std::string &data) {
        std::vector<std::string> tokens;
        boost::split(tokens, data, boost::is_any_of(","), boost::token_compress_off);
    }

    void RobotInfo::ConfigCamera(rclcpp::Node::SharedPtr node) {
        image_subscriber = std::make_shared<ImageSubscriber>(node);
    }

    bool RobotInfo::operator==(const RobotInfo &rhs) const {
        return name == rhs.name && position == rhs.position && orientation == rhs.orientation && description == rhs.description;
    }
}