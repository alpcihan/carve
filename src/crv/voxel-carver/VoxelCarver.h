#pragma once

#include "crv/video-reader/VideoReader.h"
#include "crv/utils/calib.h"
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <string>
#include <memory>

namespace crv
{
    struct VoxelCarverParams
    {
        std::string calibVideoSource = "resources/chessboard.mp4"; // camera calibration video source (for calibration with checkerboard)
        std::string sceneVideoSource = "resources/dragon.mp4"; // video source with object to carve and Aruco markers

        cv::Vec2i checkerBoardDims = {6, 9}; // vertical and horizontal grid count of the checkerboard
        float markerSize = 0.05; // size of the Aruco markers

        uint32_t calibVideoSkipBy = 24; // amount of frames to skip for each frame read from the calibration video
        uint32_t sceneVideoSkipBy = 24; // amount of frames to skip for each frame read from the main scene video

        uint32_t voxelSpaceDim = 1000; // dimension (a) of the a*a*a voxel cube space
    };

    class VoxelCarver
    {
        public:
            VoxelCarver(const VoxelCarverParams& voxelCarverParams);

            void run();
            
        private:
            VoxelCarverParams m_params;
            calib::Cam m_cam;
            std::unique_ptr<VideoReader> m_sceneVideo = nullptr;

        private:
            void _calibCamera(); 
            void _getThresholdFromTheUser();
            void _carve();
    };
}