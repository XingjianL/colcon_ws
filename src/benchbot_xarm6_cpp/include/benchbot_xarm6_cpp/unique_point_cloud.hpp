#ifndef UNIQUEPOINTCLOUD_HPP
#define UNIQUEPOINTCLOUD_HPP
#pragma once
#include <opencv2/opencv.hpp>
#include "open3d/Open3D.h"
namespace benchbot_xarm6
{
    class UniquePointCloud
    {
    public:
        UniquePointCloud();
        UniquePointCloud(std::tuple<uint8_t, uint8_t, uint8_t> segment_color);//, open3d::camera::PinholeCameraIntrinsic intrinsics);
        ~UniquePointCloud();
        std::shared_ptr<open3d::geometry::PointCloud> o3d_pc;
        std::tuple<uint8_t, uint8_t, uint8_t> segment_color;

        // nbv
        int nbv_new_points;
        uint32_t nbv_step;
        std::vector<int> nbv_optimal_order;
        std::vector<float> nbv_view_points;
        std::vector<float> nbv_view_center;
        //open3d::camera::PinholeCameraIntrinsic intrinsics_;

        bool buildPointCloud(
            cv::Mat &depth_img, cv::Mat &segment_img, cv::Mat &color_img, 
            std::tuple<uint8_t, uint8_t, uint8_t> color, 
            const Eigen::Matrix4d &transform,
            const open3d::camera::PinholeCameraIntrinsic &intrinsics_,
            std::string& save_intermediate//,
            //open3d::visualization::Visualizer &visualizer
            );
        void savePointCloud(const std::string& filepath);

    private:
        bool appendPointCloud(std::shared_ptr<open3d::geometry::PointCloud> pc, std::tuple<uint8_t, uint8_t, uint8_t> color);
        std::string filename_;
        uint32_t append_count_ = 0;
        bool pc_updated;
    };
}

#endif