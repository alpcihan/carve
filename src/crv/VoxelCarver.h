#pragma once

#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace crv
{
    struct VoxelCarverParams
    {
        std::string calibVideoSource; // camera calibration video source (for calibration with checkerboard)
        std::string sceneVideoSource; // video source with object to carve and Aruco markers
        cv::Vec2i checkerBoardDims; // vertical and horizontal grid count of the checkerboard
        float markerSize; // size of the Aruco markers
        uint32_t calibVideoSkipBy; // amount of frames to skip for each frame read from the calibration video
        uint32_t sceneVideoSkipBy; // amount of frames to skip for each frame read from the main scene video
    };

    class VoxelCarver
    {
        public:
            VoxelCarver(const VoxelCarverParams& voxelCarverParams);

            void run();
            
        private:
            VoxelCarverParams m_params;
    };
}