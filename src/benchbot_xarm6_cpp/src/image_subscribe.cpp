#include "benchbot_xarm6_cpp/image_subscribe.hpp"
//#include "image_subscribe.hpp"

namespace benchbot_xarm6 {
    ImageSubscriber::ImageSubscriber(rclcpp::Node::SharedPtr node) 
    {
        node_ = node;
        o3d_pc = std::make_shared<open3d::geometry::PointCloud>();

        publish_pcd_in_ros_ = true;
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // image_sub_color_ = node->create_subscription<sensor_msgs::msg::Image>(
        //     "/ue5/one/Color",
        //     10,
        //     std::bind(&ImageSubscriber::RGBImageCallback, this, std::placeholders::_1)
        // );
        // image_sub_segment_ = node->create_subscription<sensor_msgs::msg::Image>(
        //     "/ue5/one/Segmentation",
        //     10,
        //     std::bind(&ImageSubscriber::SegmentImageCallback, this, std::placeholders::_1)
        // );
        // image_sub_depth_ = node->create_subscription<sensor_msgs::msg::Image>(
        //     "/ue5/one/Depth",
        //     10,
        //     std::bind(&ImageSubscriber::DepthImageCallback, this, std::placeholders::_1)
        // );
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
        // Initialize Open3D visualizer
        //vis_ = std::make_shared<open3d::visualization::Visualizer>();
        //vis_->CreateVisualizerWindow("RGBD Point Cloud", 1280, 720);

    }

    ImageSubscriber::~ImageSubscriber() 
    {
        stop();
    }

    void ImageSubscriber::RGBImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg) 
    {
        //std::cout << "imageCallback-RGB" << std::endl;
        try 
        {
            cv_img_ = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
            //cv::flip(cv_img, cv_img, -1);
            {
                std::lock_guard<std::mutex> lock(image_queue_mutex_);
                image_queue_.push(cv_img_.clone());
            }
            cv::imshow("color_view", cv_img_);
            cv::waitKey(1);
            //process_to_rgbd();
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

            cv::Mat normalized_image;
            cv::threshold(cv_img_depth_, cv_img_depth_, 100.0, 0, cv::THRESH_TOZERO_INV);
            cv::normalize(cv_img_depth_, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            //cv::flip(normalized_image, normalized_image, -1);
            cv::imshow("depth_view", normalized_image);
            cv::waitKey(1);
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
            //cv::flip(cv_img, cv_img, -1);
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
        RCLCPP_INFO(node_->get_logger(), "RGBDImageCallback");
        RGBImageCallback(msg_color);
        SegmentImageCallback(msg_segment);
        DepthImageCallback(msg_depth);
        process_to_rgbd(callback_time);
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
    void ImageSubscriber::process_to_rgbd(rclcpp::Time callback_time)
    {
        if (cv_img_.empty() || cv_img_segment_.empty() || cv_img_depth_.empty())
        {
            return;
        }
        cv::Mat depth_mmeters;
        cv_img_depth_.convertTo(depth_mmeters, CV_32FC1);
        depth_mmeters *= 10.0;

        cv::Mat rgb_image;
        cv::cvtColor(cv_img_, rgb_image, cv::COLOR_BGR2RGB);

        open3d::geometry::Image o3d_rgb_image;
        o3d_rgb_image.Prepare(rgb_image.cols, rgb_image.rows, rgb_image.channels(), 1);
        std::memcpy(o3d_rgb_image.data_.data(), rgb_image.data, o3d_rgb_image.data_.size());

        open3d::geometry::Image o3d_depth_image;
        o3d_depth_image.Prepare(cv_img_depth_.cols, cv_img_depth_.rows, cv_img_depth_.channels(), 4);
        std::memcpy(o3d_depth_image.data_.data(), depth_mmeters.data, o3d_depth_image.data_.size());

        auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(o3d_rgb_image, o3d_depth_image, 1000.0, 3.0, false);
        auto intrinsics = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 272.868231007, 272.868231007, 320, 240
        );

        auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(
            *rgbd_image, intrinsics
        );
       

        //cv_img_.release();
        //cv_img_segment_.release();
        //cv_img_depth_.release();

        // Visualize point cloud
        //vis_->UpdateGeometry(pcd);
        //vis_->PollEvents();
        //open3d::io::WritePointCloud("/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/pcd/pcd.ply", *pcd);
        if (publish_pcd_in_ros_){
            pcd->VoxelDownSample(0.001);
            RCLCPP_INFO(node_->get_logger(), "pcd size: %ld", pcd->points_.size());
            auto frame_transform = tf_buffer_->lookupTransform(
                "world", "link_eef", tf2::TimePointZero
            );
            RCLCPP_INFO(node_->get_logger(), "frame_transform: %f %f %f", frame_transform.transform.translation.x, frame_transform.transform.translation.y, frame_transform.transform.translation.z);
            Eigen::Matrix4d transformation_mat;
            const auto &translation = frame_transform.transform.translation;
            const auto &rotation = frame_transform.transform.rotation;
            Eigen::Quaterniond q(rotation.w, rotation.x, rotation.y, rotation.z);
            Eigen::Matrix3d rot = q.toRotationMatrix();
            transformation_mat.block<3, 3>(0, 0) = rot;
            transformation_mat(0, 3) = translation.x;
            transformation_mat(1, 3) = translation.y;
            transformation_mat(2, 3) = translation.z;
            transformation_mat(3, 3) = 1.0;
            pcd->Transform(transformation_mat);
            RCLCPP_INFO(node_->get_logger(), "2");
            o3d_pc->points_.insert(o3d_pc->points_.end(), pcd->points_.begin(), pcd->points_.end());
            o3d_pc->colors_.insert(o3d_pc->colors_.end(), pcd->colors_.begin(), pcd->colors_.end());
            RCLCPP_INFO(node_->get_logger(), "3");
            o3d_pc->VoxelDownSample(0.005);
            RCLCPP_INFO(node_->get_logger(), "4");
            open3d::io::WritePointCloud("/home/xing2204/Lab/colcon_ws/src/benchbot_xarm6_cpp/pcd/pcd.ply", *o3d_pc); 
            RCLCPP_INFO(node_->get_logger(), "5");

            convert_to_ros_pointcloud(*o3d_pc, ros_pc, callback_time);
            publish_pcd_in_ros_ = false;
            //tf_buffer_->transform(ros_pc, ros_pc, "world",tf2::durationFromSec(0.1));
            ros_pc.header.frame_id = "world";
            point_cloud_pub_->publish(ros_pc);
            
            
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