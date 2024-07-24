#include "benchbot_xarm6_cpp/image_subscribe.hpp"
//#include "image_subscribe.hpp"

namespace benchbot_xarm6 {
    ImageSubscriber::ImageSubscriber(rclcpp::Node::SharedPtr node) 
    {
        intrinsics_ = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 272.868229679, 272.868229679, 320, 240
        );
        node_ = node;
        //o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
        //o3d_pc_vector_ = std::vector<benchbot_xarm6::UniquePointCloud>();

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        sync_sub_color_.subscribe(node_, "/ue5/one/Color");
        sync_sub_segment_.subscribe(node_, "/ue5/one/Segmentation");
        sync_sub_depth_.subscribe(node_, "/ue5/one/Depth");

        point_cloud_pub_ = node->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);

        sync_ = std::make_shared<message_filters::Synchronizer<ApproxTimeSyncPolicy>>(
            ApproxTimeSyncPolicy(10),
            sync_sub_color_, sync_sub_segment_, sync_sub_depth_
        );
        sync_->registerCallback(std::bind(&ImageSubscriber::RGBDImageCallback, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
        sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.1));
        video_writer_ = cv::VideoWriter("output.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 30, cv::Size(640, 480));

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
            cv::threshold(cv_img_depth_, cv_img_depth_, 800.0, 0, cv::THRESH_TOZERO_INV);
            cv_img_depth_.convertTo(depth_mmeters_, CV_32FC1);
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
            //uniqueColors_.clear();
            for (int y = 0; y < cv_img_segment_.rows; ++y) {
                for (int x = 0; x < cv_img_segment_.cols; ++x) {
                    cv::Vec3b color = cv_img_segment_.at<cv::Vec3b>(y, x);
                    uniqueColors_.insert(std::make_tuple(color[2], color[1], color[0])); // bgr -> rgb
                }
            }
            //cv::flip(cv_img, cv_img, -1);
            cv::imwrite("segmentation.png", cv_img_segment_);
            cv::imshow("segmentation_view", cv_img_segment_);
            cv::waitKey(1);
        }
        catch (cv_bridge::Exception& e) 
        {
            RCLCPP_ERROR(node_->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        }
    }

    void ImageSubscriber::RGBDImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg_color, const sensor_msgs::msg::Image::ConstSharedPtr& msg_segment, const sensor_msgs::msg::Image::ConstSharedPtr& msg_depth)
    {
        auto callback_time = node_->now();
        if (under_recon_) {
            return;
        }
        RCLCPP_INFO(node_->get_logger(), "RGBDImageCallback");

        RGBImageCallback(msg_color);
        SegmentImageCallback(msg_segment);
        DepthImageCallback(msg_depth);
        //process_to_rgbd(callback_time);

        auto frame_transform = tf_buffer_->lookupTransform(
            "world", "link_eef", tf2::TimePointZero
        );
        const auto &translation = frame_transform.transform.translation;
        const auto &rotation = frame_transform.transform.rotation;
        Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
        Eigen::Matrix3d rot = q.toRotationMatrix();
        transformation_mat_.block<3, 3>(0, 0) = rot;
        transformation_mat_(0, 3) = translation.x;
        transformation_mat_(1, 3) = translation.y;
        transformation_mat_(2, 3) = translation.z;
        transformation_mat_(3, 3) = 1.0;
    }

    void ImageSubscriber::start()
    {
        executor_.add_node(node_);
        executor_thread_ = std::thread([this]() { executor_.spin(); });


        // Start visualization thread
        //vis_thread_ = std::thread([this]() { vis_->Run(); });
    }

    void ImageSubscriber::stop()
    {
        executor_.cancel();
        if (executor_thread_.joinable())
        {
            executor_thread_.join();
        }
        //vis_->Close();
        //vis_thread_.join();
        video_writer_.release();

        cv::destroyAllWindows();
    }
    bool ImageSubscriber::process_to_pc(std::vector<UniquePointCloud>& unique_pcs, uint8_t instance_id)
    {
        if (rgb_image_.empty() || cv_img_segment_.empty() || depth_mmeters_.empty())
        {
            return false;
        }
        auto rgb_image = rgb_image_.clone();
        
        auto depth_mmeters = depth_mmeters_.clone();
        depth_mmeters *= 10.0;
        // build point cloud for each unique color
        for (auto uniqueColor : uniqueColors_){
            bool added = false;
            if (std::get<2>(uniqueColor) != instance_id) {
                //RCLCPP_INFO(node_->get_logger(), "color %d %d %d not for instance %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor), instance_id);
                continue;
            }
            for (auto& unique_pc : unique_pcs) {
                
                added = unique_pc.buildPointCloud(depth_mmeters, cv_img_segment_, rgb_image, uniqueColor, transformation_mat_);
                if (added) {
                    RCLCPP_INFO(node_->get_logger(), 
                        "color %d %d %d added for instance %d", 
                        std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor), 
                        instance_id);
                    break;
                }
            }
            // new point cloud if not added (point cloud with same color doesn't exist)
            if (!added){
                RCLCPP_INFO(node_->get_logger(), "adding new point cloud for color: %d %d %d", std::get<0>(uniqueColor), std::get<1>(uniqueColor), std::get<2>(uniqueColor));
                unique_pcs.push_back(benchbot_xarm6::UniquePointCloud(uniqueColor, intrinsics_));
                added = unique_pcs.back().buildPointCloud(depth_mmeters, cv_img_segment_, rgb_image, uniqueColor, transformation_mat_);
            }
            //RCLCPP_INFO(node_->get_logger(), "num of point clouds: %ld", o3d_pc_vector_.size());
            //convert_to_ros_pointcloud(*o3d_pc, ros_pc, callback_time);
        }
        return true;
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