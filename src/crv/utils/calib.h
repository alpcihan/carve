#pragma once

#include "crv/video-reader/VideoReader.h"
#include <Eigen/Dense>

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
            cv::Vec3d rVec, tVec;
            cv::Matx44d transform;
            cv::Mat image;
            bool isValid = true;
        };

        void estimateCamMatrixAndDistortion(VideoReader &video, const cv::Vec2i &checkerBoardDims, Cam &out);
        void estimateCamToARBoard(const cv::Mat &image, const Cam &cam, CamToBoardData& out);
        void evaluateUndistortedImage(const cv::Mat &image, cv::Mat& out, const Cam &cam);
    }
}