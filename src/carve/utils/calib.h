#pragma once

#include "carve/shared.h"

namespace crv
{
    class VideoReader;

    namespace calib
    {
        void estimateCamMatrixAndDistortion(VideoReader &video, const cv::Vec2i &checkerBoardDims, cv::Matx33d& intrinsics, cv::Mat& distCoeffs);
        bool estimateCamToArUcoBoard(const cv::Mat &image, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, cv::Vec3d& rVec, cv::Vec3d& tVec);
    }
}