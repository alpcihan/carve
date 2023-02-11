#pragma once

#include "carve/shared.h"

namespace crv
{
    struct VoxelCarverParams
    {
        uint32_t voxelSpaceDim = 100;                              // dimension (a) of the 3D a*a*a voxel cube space
        float volumeScale = 1.0;                                   // scale of the voxel cube space
    };

    class VoxelCarver
    {
    public:
        VoxelCarver(const VoxelCarverParams &voxelCarverParams);

        bool get(uint32_t i) const { return m_space[i] == _VoxelState::CARVED; }
        bool get(uint32_t x, uint32_t y, uint32_t z) const { return get((z * m_params.voxelSpaceDim + y) * m_params.voxelSpaceDim + x); }
        
        void carveByBinaryImage(const cv::Mat& frame, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec);
        void saveCurrentStateAsPLY(const std::string& fileName) const;
        void saveCurrentStateAsMesh(const std::string& fileName) const;

    private: 
        // enum class _VoxelState: 
        // if the voxel is carved, it stores the value "CARVED".
        // if a voxel is not carved and it do not have a normal vector estimation, it stores the value "NOT_CARVED".
        // if a voxel is not carved and it has a normal vector estimation, it represents the estimated normal vector 
        //    (with six states: UP, BOTTOM, FRONT, BACK, RIGHT, LEFT) of the voxel with respect to the voxel space boundaries.
        enum class _VoxelState : uint8_t {CARVED, NOT_CARVED, UP, BOTTOM, FRONT, BACK, RIGHT, LEFT};

    private:
        VoxelCarverParams m_params;
        std::vector<_VoxelState> m_space;                         // TODO: replace with a more efficient solution
        uint32_t m_pointCount;

    private:
        void _initVoxelSpace(); 
        void _carveByBinaryImage(const cv::Mat& binaryImage, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, const cv::Vec3d& rVec, const cv::Vec3d& tVec);      
    };
}