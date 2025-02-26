#include "benchbot_xarm6_cpp/image_subscribe.hpp"
#include <filesystem>
//#include "image_subscribe.hpp"

namespace benchbot_xarm6 {
    ImageSubscriber::ImageSubscriber(std::string &node_name, double camera_FOV, int width, int height, bool capture_both, std::string &topic_name) 
    {
        executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

        node_ = std::make_shared<rclcpp::Node>(
            node_name,
            rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
        );
        intrinsics_ = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 272.868229679, 272.868229679, 320, 240
        );
        
        
        sync_sub_color_.subscribe(node_, "/ue5/"+topic_name+"/Color");
        sync_sub_color1_.subscribe(node_, "/ue5/"+topic_name+"/ColorOne");

        sync_sub_segment_.subscribe(node_, "/ue5/"+topic_name+"/Segmentation");
        
        sync_sub_depth_.subscribe(node_, "/ue5/"+topic_name+"/Depth");
        sync_sub_depth1_.subscribe(node_, "/ue5/"+topic_name+"/DepthOne");

        point_cloud_pub_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        stereo_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyStereo>>(
            ApproxTimeSyncPolicyStereo(1),
            sync_sub_color_, sync_sub_color1_, sync_sub_depth_, sync_sub_depth1_
        );
        stereo_sync_->registerCallback(
            std::bind(&ImageSubscriber::StereoImageCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,std::placeholders::_4));
        rgbd_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyRGBD>>(
            ApproxTimeSyncPolicyRGBD(1),
            sync_sub_color_, sync_sub_segment_, sync_sub_depth_
        );
        rgbd_sync_->registerCallback(
            std::bind(&ImageSubscriber::RGBDImageCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));

        //sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.125));
        video_writer_ = cv::VideoWriter("output.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(640, 480));
        waiting_msg_rgbd = true;
        waiting_msg_stereo = true;
        under_recon_ = false;
        capture_both_ = capture_both;
        update_intrinsics(camera_FOV, width, height);
    }

    ImageSubscriber::~ImageSubscriber() 
    {
        stop();
    }

    void ImageSubscriber::RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        try 
        {
            cv_img_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            
            cv::cvtColor(cv_img_, rgb_image_, cv::COLOR_BGR2RGB);
            {
                std::lock_guard<std::mutex> lock(image_queue_mutex_);
                image_queue_.push(cv_img_.clone());
            }
            // cv::imshow("color_view", cv_img_);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }
    void ImageSubscriber::DepthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        //std::cout << "imageCallback-Depth" << std::endl;
        try 
        {
            cv_img_depth_ = cv_bridge::toCvShare(msg, "32FC1")->image.clone();

            //cv::Mat normalized_image;
            //cv::threshold(cv_img_depth_, cv_img_depth_, 300.0, 0, cv::THRESH_TOZERO_INV);
            // cv_img_depth_.convertTo(depth_cmeters_, CV_32FC1);
            // cv::normalize(cv_img_depth_, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            // //cv::flip(normalized_image, normalized_image, -1);
            // cv::imshow("depth_view", normalized_image);
            // cv::waitKey(1);
            //process_to_rgbd();
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to '32FC1'.", msg->encoding.c_str());
        }
    }

    void ImageSubscriber::SegmentImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        //std::cout << "imageCallback-Segmentation" << std::endl;
        try 
        {
            cv_img_segment_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            uniqueColors_.clear();
            for (int y = 0; y < cv_img_segment_.rows; ++y) {
                for (int x = 0; x < cv_img_segment_.cols; ++x) {
                    cv::Vec3b color = cv_img_segment_.at<cv::Vec3b>(y, x);
                    uniqueColors_.insert(std::make_tuple(color[2], color[1], color[0])); // bgr -> rgb
                }
            }
            //cv::flip(cv_img, cv_img, -1);
            // cv::imwrite("segmentation.png", cv_img_segment_);
            // cv::imshow("segmentation_view", cv_img_segment_);
            // cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void ImageSubscriber::RGBDImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, const sensor_msgs::msg::Image::ConstSharedPtr& msg_segment, const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth)
    {
        waiting_msg_rgbd = false;
        if (under_recon_) {
            //RCLCPP_INFO(node_->get_logger(), "Recon is under progress, not processing the received image.");
            return;
        }
        
        //RCLCPP_INFO(node_->get_logger(), "RGBDImageCallback");
        RGBImageCallback(msg_color);
        SegmentImageCallback(msg_segment);
        DepthImageCallback(msg_depth);
        waiting_msg_rgbd = false;
        
    }

    void ImageSubscriber::StereoImageCallback(
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_color1, 
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth,
            const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth1)
    {
        //RCLCPP_INFO(node_->get_logger(), "StereoImageCallback");
        cv_img_ = cv_bridge::toCvShare(msg_color, "bgr8")->image.clone();
        cv_img1_ = cv_bridge::toCvShare(msg_color1, "bgr8")->image.clone();
        cv_img_depth_ = cv_bridge::toCvShare(msg_depth, "32FC1")->image.clone();
        cv_img1_depth_ = cv_bridge::toCvShare(msg_depth1, "32FC1")->image.clone();
        waiting_msg_stereo = false;
    }

    void ImageSubscriber::start()
    {
        executor_->add_node(node_);
        executor_thread_ = std::thread([this]() { executor_->spin(); });
        waiting_for_sync();
    }

    void ImageSubscriber::stop()
    {
        executor_->cancel();
        if (executor_thread_.joinable()) {
           executor_thread_.join();
        }
        video_writer_.release();
        cv::destroyAllWindows();
    }

    void ImageSubscriber::update_intrinsics(double fov, int width, int height)
    {
        video_writer_.release();
        video_writer_ = cv::VideoWriter("output.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), 30, cv::Size(width, height));

        double fx = (width / tan((fov*M_PI/180.0)/2.0)) / 2;
        double fy = fx;
        RCLCPP_INFO(node_->get_logger(), "fx: %f", fx);
        intrinsics_ = open3d::camera::PinholeCameraIntrinsic(width, height, fx, fy, width / 2, height / 2);
    }

    void ImageSubscriber::waiting_for_sync()
    {
        //clear_images();
        //RCLCPP_INFO(node_->get_logger(), "clearing image topics");
        std::unique_lock<std::mutex> lock(waiting_msg_mutex);
        lock.unlock();
        rgbd_sync_.reset();
        stereo_sync_.reset();
        //RCLCPP_INFO(node_->get_logger(), "clearing stereo topics");
        stereo_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyStereo>>(
            ApproxTimeSyncPolicyStereo(1),
            sync_sub_color_, sync_sub_color1_, sync_sub_depth_, sync_sub_depth1_
        );
        stereo_sync_->registerCallback(
            std::bind(&ImageSubscriber::StereoImageCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,std::placeholders::_4));

        //RCLCPP_INFO(node_->get_logger(), "clearing rgbd topics");
        rgbd_sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicyRGBD>>(
            ApproxTimeSyncPolicyRGBD(1),
            sync_sub_color_, sync_sub_segment_, sync_sub_depth_
        );
        rgbd_sync_->registerCallback(
            std::bind(&ImageSubscriber::RGBDImageCallback, this, 
                std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        waiting_msg_rgbd = true;
        waiting_msg_stereo = true;
        lock.lock();
        while((capture_both_ && (waiting_msg_rgbd || waiting_msg_stereo)) ||
            (!capture_both_ && (waiting_msg_rgbd && waiting_msg_stereo))){
            //RCLCPP_INFO(node_->get_logger(), "waiting for sync - Image");
            lock.unlock();
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            //executor_->spin_some();
            lock.lock();
        }
    }
    void ImageSubscriber::clear_images()
    {
        rgb_image_.release();
        cv_img1_.release();
        cv_img_.release();
        cv_img1_depth_.release();
        cv_img_segment_.release();
        cv_img_depth_.release();
    }

    bool ImageSubscriber::process_to_pc(
        std::vector<UniquePointCloud>& unique_pcs, 
        const Eigen::Matrix4d &transform,
        uint8_t instance_id_g, uint8_t instance_id_b,
        std::string& save_intermediate//,
        //open3d::visualization::Visualizer& visualizer
        )
    {
        if (rgb_image_.empty() || cv_img_segment_.empty() || cv_img_depth_.empty()) {
            RCLCPP_WARN(node_->get_logger(), "Images empty while reconstructing for instance: %d, %d", instance_id_g, instance_id_b);
            return false;
        }

        auto rgb_image = rgb_image_.clone();
        auto seg_image = cv_img_segment_.clone();
        auto depth_cmeters = cv_img_depth_.clone();
        // build point cloud for each unique color
        for (auto uniqueColor : uniqueColors_)
        {
            bool added = false;
            if (std::get<2>(uniqueColor) != instance_id_b || std::get<1>(uniqueColor) != instance_id_g) {
                //RCLCPP_INFO(node_->get_logger(), "color %d %d %d not for instance %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor), instance_id);
                continue;
            }
            for (auto& unique_pc : unique_pcs) {
                
                added = unique_pc.buildPointCloud(
                    depth_cmeters, seg_image, rgb_image, 
                    uniqueColor, 
                    transform,
                    intrinsics_,
                    save_intermediate//,
                    //visualizer
                    );
                if (added) 
                {
                    // RCLCPP_INFO(node_->get_logger(), 
                    //     "color %d %d %d added for instance %d %d", 
                    //     std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor), 
                    //     instance_id_g, instance_id_b);
                    break;
                }
            }
            // new point cloud if not added (point cloud with same color doesn't exist)
            if (!added)
            {
                RCLCPP_INFO(node_->get_logger(), "adding new point cloud for color: %d %d %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor));
                unique_pcs.push_back(benchbot_xarm6::UniquePointCloud(uniqueColor));
                added = unique_pcs.back().buildPointCloud(
                    depth_cmeters, cv_img_segment_, rgb_image, 
                    uniqueColor, 
                    transform,
                    intrinsics_,
                    save_intermediate//,
                    //visualizer
                    );
                RCLCPP_DEBUG(rclcpp::get_logger("benchbot_debug"), "new pcd added: %d %d %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor));
            }
            //RCLCPP_INFO(node_->get_logger(), "num of point clouds: %ld", o3d_pc_vector_.size());
            //convert_to_ros_pointcloud(*o3d_pc, ros_pc, callback_time);
        }
        return true;
    }

    void ImageSubscriber::save_images(std::string path)
    {
        std::filesystem::create_directories(path);
        if (!cv_img_.empty()){
            cv::imwrite(path + "/rgb_" + std::to_string(capture_count_) + ".png", cv_img_);
        }
        if (!cv_img1_.empty()){
            cv::imwrite(path + "/rgb1_" + std::to_string(capture_count_) + ".png", cv_img1_);
        }
        if (!cv_img_segment_.empty()){
            cv::imwrite(path + "/seg_" + std::to_string(capture_count_) + ".png", cv_img_segment_);
        }
        if (!cv_img_depth_.empty()){
            cv::imwrite(path + "/depth_" + std::to_string(capture_count_) + ".exr", cv_img_depth_);
        }
        if (!cv_img1_depth_.empty()){
            cv::imwrite(path + "/depth1_" + std::to_string(capture_count_) + ".exr", cv_img1_depth_);
        }
    }

    void ImageSubscriber::convert_to_ros_pointcloud(const open3d::geometry::PointCloud &pc, sensor_msgs::msg::PointCloud2 &msg, rclcpp::Time callback_time)
    {
        msg.header.frame_id = "link_eef";
        msg.header.stamp = callback_time;
        msg.height = 1;
        msg.width = pc.points_.size();
        msg.is_dense = false;
        msg.is_bigendian = false;
        
        sensor_msgs::PointCloud2Modifier modifier(msg);
        modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
        modifier.resize(pc.points_.size());

        sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(msg, "r");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(msg, "g");
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(msg, "b");

        for (size_t i = 0; i < pc.points_.size(); ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_r, ++iter_g, ++iter_b)
        {
            const Eigen::Vector3d &point = pc.points_[i];
            const Eigen::Vector3d &color = pc.colors_[i];

            *iter_x = point.x();
            *iter_y = point.y();
            *iter_z = point.z();
            *iter_r = static_cast<uint8_t>(color.x() * 255.0);
            *iter_g = static_cast<uint8_t>(color.y() * 255.0);
            *iter_b = static_cast<uint8_t>(color.z() * 255.0);
        }
    }
}