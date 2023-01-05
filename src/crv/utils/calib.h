#pragma once

#include "crv/shared.h"
#include "crv/video-reader/VideoReader.h"

namespace crv
{
    namespace calib
    {
        struct Cam
        {
            cv::Mat distCoeffs;
            cv::Matx33d cameraMatrix;
            cv::Matx33d optimizedCameraMatrix;
        };

        struct CamToBoardData
        {
            cv::Vec3d rVec;
            cv::Vec3d tVec;
        };

        void estimateCamMatrixAndDistortion(VideoReader &video, const cv::Vec2i &checkerBoardDims, Cam &out);
        bool estimateCamToARBoard(const cv::Mat &image, const Cam &cam, CamToBoardData& out);
        void calculateUndistortedImage(const cv::Mat &image, const Cam &cam, cv::Mat& out);
    }
}