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
        this->intrinsics = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 272.868229679, 272.868229679, 320, 240
        );
    }

    UniquePointCloud::UniquePointCloud(std::tuple<uint8_t, uint8_t, uint8_t> segment_color, open3d::camera::PinholeCameraIntrinsic intrinsics)
    {
        o3d_pc = std::make_shared<open3d::geometry::PointCloud>();
        this->segment_color = segment_color;
        this->intrinsics = intrinsics;
        this->intrinsics = open3d::camera::PinholeCameraIntrinsic(
            640, 480, 272.868229679, 272.868229679, 320, 240
        );
    }

    UniquePointCloud::~UniquePointCloud()
    {
    }

    bool UniquePointCloud::buildPointCloud(cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, std::tuple<uint8_t, uint8_t, uint8_t> color, Eigen::Matrix4d transform)
    {
        if (color != segment_color)
        {
            return false;
        }
        auto masked_image = segment_img.clone();
        cv::inRange(
            masked_image, 
            cv::Scalar(std::get<2>(segment_color), std::get<1>(segment_color), std::get<0>(segment_color)), 
            cv::Scalar(std::get<2>(segment_color), std::get<1>(segment_color), std::get<0>(segment_color)), 
            masked_image);
        auto depth_masked = depth_img.clone();
        masked_image.convertTo(masked_image, depth_img.type());
        cv::multiply(depth_masked, masked_image, depth_masked, 1.0 / 255.0);

        cv::Mat normalized_image;
        cv::normalize(depth_masked, normalized_image, 0, 255, cv::NORM_MINMAX, CV_8UC1);
            //cv::flip(normalized_image, normalized_image, -1);
        cv::imshow("seg_mask_view", masked_image);
        cv::imshow("seg_depth_view", normalized_image);
        cv::waitKey(100);

        open3d::geometry::Image o3d_rgb_image;
        o3d_rgb_image.Prepare(color_img.cols, color_img.rows, color_img.channels(), 1);
        std::memcpy(o3d_rgb_image.data_.data(), color_img.data, o3d_rgb_image.data_.size());

        open3d::geometry::Image o3d_depth_image;
        o3d_depth_image.Prepare(depth_masked.cols, depth_masked.rows, depth_masked.channels(), 4);
        std::memcpy(o3d_depth_image.data_.data(), depth_masked.data, o3d_depth_image.data_.size());

        auto rgbd_image = open3d::geometry::RGBDImage::CreateFromColorAndDepth(o3d_rgb_image, o3d_depth_image, 1000.0, 5.0, false);

        auto pcd = open3d::geometry::PointCloud::CreateFromRGBDImage(
            *rgbd_image, this->intrinsics
        );

        pcd->Transform(transform);

        return appendPointCloud(pcd, segment_color);

    }

    bool UniquePointCloud::appendPointCloud(std::shared_ptr<open3d::geometry::PointCloud> pc, std::tuple<uint8_t, uint8_t, uint8_t> color)
    {
        if (color != segment_color) {
            return false;
        }
        //std::cout << "appendPointCloud" << std::endl;
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
        std::string filename = 
            std::to_string(std::get<0>(segment_color)) + "_" + 
            std::to_string(std::get<1>(segment_color)) + "_" + 
            std::to_string(std::get<2>(segment_color)) + ".ply";
        if (o3d_pc->points_.size() < 500) {
            return;
        }
        std::filesystem::path path(filepath);
        if (!std::filesystem::exists(path)) {
            std::filesystem::create_directories(path);
        }
        
        std::cout << "savePointCloud: " << filename << " " << o3d_pc->points_.size() << std::endl;
        open3d::io::WritePointCloud(filepath + filename, *o3d_pc);
    }
}