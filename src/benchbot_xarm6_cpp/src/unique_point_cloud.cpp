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
        // intrinsics_ = open3d::camera::PinholeCameraIntrinsic(
        //     640, 480, 272.868229679, 272.868229679, 320, 240
        // );
        filename_ = 
            std::to_string(std::get<0>(segment_color)) + "_" + 
            std::to_string(std::get<1>(segment_color)) + "_" + 
            std::to_string(std::get<2>(segment_color)) + ".pcd";
    }

    UniquePointCloud::UniquePointCloud(std::tuple<uint8_t, uint8_t, uint8_t> segment_color)//, open3d::camera::PinholeCameraIntrinsic intrinsics)
    {
        o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
        this->segment_color = segment_color;
        // intrinsics_ = intrinsics;
        // intrinsics_ = open3d::camera::PinholeCameraIntrinsic(
        //     640, 480, 272.868229679, 272.868229679, 320, 240
        // );
        filename_ = 
            std::to_string(std::get<0>(segment_color)) + "_" + 
            std::to_string(std::get<1>(segment_color)) + "_" + 
            std::to_string(std::get<2>(segment_color)) + ".pcd";
    }

    UniquePointCloud::~UniquePointCloud()
    {
    }

    bool UniquePointCloud::buildPointCloud(
        cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, 
        std::tuple<uint8_t, uint8_t, uint8_t> color, 
        const Eigen::Matrix4d & transform,
        const open3d::camera::PinholeCameraIntrinsic &intrinsics_,
        std::string& save_intermediate)
    {
        if (color != segment_color)
        {
            return false;
        }
        auto masked_image = segment_img.clone();
        cv::inRange(
            segment_img, 
            cv::Scalar(std::get<2>(segment_color), std::get<1>(segment_color), std::get<0>(segment_color)), 
            cv::Scalar(std::get<2>(segment_color), std::get<1>(segment_color), std::get<0>(segment_color)), 
            masked_image);

        cv::Mat depth_masked = cv::Mat::zeros(depth_img.size(), depth_img.type());
        //masked_image.convertTo(masked_image, depth_img.type());
        //cv::multiply(depth_masked, masked_image, depth_masked, 1.0 / 255.0);
        cv::Mat mask;
        masked_image.convertTo(mask, CV_8U);
        //cv::bitwise_and(depth_masked, depth_masked, depth_masked, masked_image);
        depth_img.copyTo(depth_masked, mask);
        
        // cv::Mat normalized_image;
        // cv::normalize(depth_masked, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
        //     //cv::flip(normalized_image, normalized_image, -1);
        // cv::imshow("seg_mask_view", masked_image);
        // cv::imshow("seg_depth_view", normalized_image);
        // cv::waitKey(10);

        open3d::geometry::Image o3d_rgb_image;
        o3d_rgb_image.Prepare(color_img.cols, color_img.rows, color_img.channels(), 1);
        std::memcpy(o3d_rgb_image.data_.data(), color_img.data, o3d_rgb_image.data_.size());

        open3d::geometry::Image o3d_depth_image;
        //std::cout << depth_masked.channels() << std::endl;
        o3d_depth_image.Prepare(depth_masked.cols, depth_masked.rows, depth_masked.channels(), 4);
        std::memcpy(o3d_depth_image.data_.data(), depth_masked.data, o3d_depth_image.data_.size());

        // MARK: try 100 here
        auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(o3d_rgb_image, o3d_depth_image, 100.0, 10.0, false);

        auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(
            *rgbd_image, intrinsics_
        );

        pcd->Transform(transform);

        if (!save_intermediate.empty())
        {
            std::string filepath = save_intermediate + "/" + std::to_string(append_count_) + "_" + filename_;
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
        //std::cout << "appendPointCloud: " << pc->points_.size() << std::endl;
        o3d_pc->points_.insert(o3d_pc->points_.end(), pc->points_.begin(), pc->points_.end());
        o3d_pc->colors_.insert(o3d_pc->colors_.end(), pc->colors_.begin(), pc->colors_.end());
        // if (o3d_pc->points_.size() > 50000)
        // {
        //     o3d_pc->VoxelDownSample(0.005);
        // }
        return true;
    }

    void UniquePointCloud::savePointCloud(std::string& filepath)
    {
        // std::string filename = 
        //     std::to_string(std::get<0>(segment_color)) + "_" + 
        //     std::to_string(std::get<1>(segment_color)) + "_" + 
        //     std::to_string(std::get<2>(segment_color)) + ".ply";
        if (o3d_pc->points_.size() < 100) {
            std::cout << "not enough points to save: " << filename_ << " " << o3d_pc->points_.size() << std::endl;
            return;
        }
        std::filesystem::path path(filepath);
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }
        
        std::cout << "savePointCloud: " << filename_ << " " << o3d_pc->points_.size() << std::endl;
        open3d::io::WritePointCloudOption o3d_option(true);
        open3d::io::WritePointCloud(filepath + filename_, *o3d_pc, o3d_option);
    }
}