#pragma once

#include "carve/shared.h"
#include "carve/video-reader/VideoReader.h"

namespace crv
{
    namespace calib
    {
        void estimateCamMatrixAndDistortion(VideoReader &video, const cv::Vec2i &checkerBoardDims, cv::Matx33d& intrinsics, cv::Mat& distCoeffs);
        bool estimateCamToArUcoBoard(const cv::Mat &image, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs, cv::Vec3d& rVec, cv::Vec3d& tVec);
        // void calculateUndistortedImage(const cv::Mat &image, const Cam &cam, cv::Mat& out);
    }
}