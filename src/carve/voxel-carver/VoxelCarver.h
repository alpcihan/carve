#pragma once

#include "carve/shared.h"
#include "carve/video-reader/VideoReader.h"
#include "carve/utils/calib.h"

namespace crv
{
    struct VoxelCarverParams
    {
        std::string calibVideoSource = "resources/chessboard.mp4"; // camera calibration video source (for calibration with checkerboard)
        std::string sceneVideoSource = "resources/dragon.mp4";     // video source with object to carve and Aruco markers
        cv::Vec2i checkerBoardDims = {6, 9};                       // vertical and horizontal grid count of the checkerboard
        uint32_t calibVideoSkipBy = 24;                            // amount of frames to skip for each frame read from the calibration video
        uint32_t sceneVideoSkipBy = 24;                            // amount of frames to skip for each frame read from the main scene video
        uint32_t voxelSpaceDim = 100;                              // dimension (a) of the a*a*a voxel cube space
        double volumeScale = 1.0;                                  // size of the voxel cube space
    };

    class VoxelCarver
    {
    public:
        VoxelCarver(const VoxelCarverParams &voxelCarverParams);
        
        void run();
        void saveAsPLY(const std::string& fileName = "output.ply");

    private:
        VoxelCarverParams m_params;
        calib::Cam m_cam;
        std::unique_ptr<VideoReader> m_sceneVideo = nullptr;
        std::vector<std::vector<std::vector<bool>>> m_space; // TODO: replace with a more efficient solution
        uint32_t m_pointCount;

    private:
        void _initVoxelSpace(); 
        void _calibCamera();
        void _carveSegmentedImage(const cv::Mat &image, const calib::CamToBoardData &camToBoardData);  
    };
}