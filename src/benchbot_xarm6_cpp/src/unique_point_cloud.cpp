#include "benchbot_xarm6_cpp/unique_point_cloud.hpp"
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"
#include <filesystem>
#include <iostream>
namespace benchbot_xarm6
{
    UniquePointCloud::UniquePointCloud()
    {
        o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
        this->segment_color = std::make_tuple(0, 0, 0);
        filename_ = 
            std::to_string(std::get<0>(segment_color)) + "_" + 
            std::to_string(std::get<1>(segment_color)) + "_" + 
            std::to_string(std::get<2>(segment_color)) + ".pcd";
        pc_updated = false;
        nbv_new_points = 0;
    }

    UniquePointCloud::UniquePointCloud(std::tuple<uint8_t, uint8_t, uint8_t> segment_color)//, open3d::camera::PinholeCameraIntrinsic intrinsics)
    {
        o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
        this->segment_color = segment_color;
        filename_ = 
            std::to_string(std::get<0>(segment_color)) + "_" + 
            std::to_string(std::get<1>(segment_color)) + "_" + 
            std::to_string(std::get<2>(segment_color)) + ".pcd";
        pc_updated = false;
        nbv_new_points = 0;


    }

    UniquePointCloud::~UniquePointCloud()
    {
    }

    bool UniquePointCloud::buildPointCloud(
        cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, 
        std::tuple<uint8_t, uint8_t, uint8_t> color, 
        const Eigen::Matrix4d & transform,
        const open3d::camera::PinholeCameraIntrinsic &intrinsics_,
        std::string& save_intermediate//,
        //open3d::visualization::Visualizer& visualizer
        )
    {
        if (color != segment_color) {
            return false;
        }
        auto masked_image = segment_img.clone();
        cv::inRange(
            segment_img, 
            cv::Scalar(std::get<2>(segment_color), std::get<1>(segment_color), std::get<0>(segment_color)), 
            cv::Scalar(std::get<2>(segment_color), std::get<1>(segment_color), std::get<0>(segment_color)), 
            masked_image);

        cv::Mat depth_masked = cv::Mat::zeros(depth_img.size(), depth_img.type());

        cv::Mat mask;
        masked_image.convertTo(mask, CV_8U);
        depth_img.copyTo(depth_masked, mask);

        if (depth_masked.type() != CV_32FC1) {
            std::cerr << "Incorrect depth type" << std::endl;
        }

        //std::cout << "creating o3d_rgb" << std::endl;
        open3d::geometry::Image o3d_rgb_image;
        o3d_rgb_image.Prepare(color_img.cols, color_img.rows, color_img.channels(), 1);
        std::memcpy(o3d_rgb_image.data_.data(), color_img.data, o3d_rgb_image.data_.size());
        
        //std::cout << "creating o3d_depth" << std::endl;
        open3d::geometry::Image o3d_depth_image;
        o3d_depth_image.Prepare(depth_masked.cols, depth_masked.rows, depth_masked.channels(), 4);
        std::memcpy(o3d_depth_image.data_.data(), depth_masked.data, o3d_depth_image.data_.size());
        
        std::cout << "creating o3d_rgbd ... ";
        auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(o3d_rgb_image, o3d_depth_image, 100.0, 1.5, false);
        // visualizer.ClearGeometries();
        // visualizer.AddGeometry(rgbd_image);
        // visualizer.UpdateGeometry();
        // visualizer.PollEvents();
        // visualizer.UpdateRender();
        
        //visualizer.Run();
        std::cout << "creating o3d_pcd ... " << std::endl;
        auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(
            *rgbd_image, intrinsics_
        );
        std::cout << "built PCD: " << pcd->points_.size() << std::endl;
        // if (pcd->points_.size() > 20000) {
        //     // pcd = pcd->RandomDownSample(0.1);
        //     //std::cout << "buildPCD(randDownsample): " << pcd->points_.size() << std::endl;
        // }
        if (pcd->points_.size() == 0) {
            return true;
        }
        pcd->Transform(transform);

        // save point cloud of this particular rgbd image
        if (!save_intermediate.empty() && pcd->points_.size() > 200)
        {
            std::string filepath = save_intermediate + "_Intermediate_" + std::to_string(append_count_) + "_" + filename_;
            open3d::io::WritePointCloudOption o3d_option(true);
            open3d::io::WritePointCloud(filepath, *pcd, o3d_option);
            append_count_ += 1;
        }

        return appendPointCloud(pcd, segment_color);

    }

    bool UniquePointCloud::appendPointCloud(
        std::shared_ptr<open3d::geometry::PointCloud> pc, 
        std::tuple<uint8_t, uint8_t, uint8_t> color)
    {
        if (color != segment_color) {
            return false;
        }
        if (pc->points_.size() < 200) {
            return true;
        }
        //std::cout << "appendPointCloud: " << pc->points_.size() << std::endl;
        pc_updated = true;
        o3d_pc->points_.insert(o3d_pc->points_.end(), pc->points_.begin(), pc->points_.end());
        o3d_pc->colors_.insert(o3d_pc->colors_.end(), pc->colors_.begin(), pc->colors_.end());
        // if (o3d_pc->points_.size() > 200'000) {
        //     o3d_pc = o3d_pc->RandomDownSample(0.2);
        // }
        if (o3d_pc->points_.size() > 200'000) {
            o3d_pc = o3d_pc->VoxelDownSample(0.001);
        }
        
        return true;
    }

    void UniquePointCloud::savePointCloud(const std::string& filepath)
    {
        if (o3d_pc->points_.size() < 1280) {
            //std::cout << "not enough points to save: " << filename_ << " " << o3d_pc->points_.size() << std::endl;
            return;
        }
        if (!pc_updated){
            return;
        }
        pc_updated = false;
        std::filesystem::path path(filepath);
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }
        
        //std::cout << "savePointCloud: " << filename_ << " " << o3d_pc->points_.size() << std::endl;
        open3d::io::WritePointCloudOption o3d_option(true);
        o3d_pc = o3d_pc->VoxelDownSample(0.001);
        open3d::io::WritePointCloud(filepath + "_FinalComplete_" + filename_, *o3d_pc, o3d_option);
    }
}