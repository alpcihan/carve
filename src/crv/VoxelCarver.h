#pragma once

#include <string>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace crv
{
    struct VoxelCarverParams
    {
        std::string calibVideoSource = "chessboard.mp4"; // camera calibration video source (for calibration with checkerboard)
        std::string sceneVideoSource = "dragon.mp4"; // video source with object to carve and Aruco markers
        cv::Vec2i checkerBoardDims = {6, 9}; // vertical and horizontal grid count of the checkerboard
        float markerSize = 0.05; // size of the Aruco markers
        uint32_t calibVideoSkipBy = 24; // amount of frames to skip for each frame read from the calibration video
        uint32_t sceneVideoSkipBy = 24; // amount of frames to skip for each frame read from the main scene video
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