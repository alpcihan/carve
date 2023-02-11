#pragma once

#include "carve/shared.h"
#include "carve/video-reader/VideoReader.h"
#include "carve/utils/calib.h"

namespace crv
{
    struct VoxelCarverParams
    {
        uint32_t voxelSpaceDim = 100;                              // dimension (a) of the 3D a*a*a voxel cube space
        double volumeScale = 1.0;                                  // size of the voxel cube space
    };

    class VoxelCarver
    {
    public:
        VoxelCarver(const VoxelCarverParams &voxelCarverParams);
        
        void carveByBinaryImage(const cv::Mat& frame, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec);

        void saveCurrentStateAsPLY(const std::string& fileName = "output.ply");

        void marche(const std::string& fileName);

    private:
        VoxelCarverParams m_params;

        std::vector<bool> m_space;                                // TODO: replace with a more efficient solution //think about mesh writing function
        uint32_t m_pointCount;

    private:
        void _initVoxelSpace(); 
        void _carveByBinaryImage(const cv::Mat& binaryImage, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec);  
    };
}