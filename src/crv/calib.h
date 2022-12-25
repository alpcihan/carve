#pragma once

#include "crv/VideoReader.h"
#include <Eigen/Dense>

namespace crv
{
    namespace calib
    {
        struct CameraToMarkerData
        {
            struct Data
            {
                uint32_t markerId;
                cv::Matx44d transform;
            };
            
            std::vector<Data> data;
            cv::Mat image; 
        };

        void estimateCameraMatrixAndDistortion(VideoReader &video, const cv::Vec2i& checkerBoardDims, cv::Mat& cameraMatrix, cv::Mat& distCoeffs);
        bool estimateCameraToMarkers(const cv::Mat& image, const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs, CameraToMarkerData& data, float markerSize = 0.05, bool storeImage = false);
    }
}