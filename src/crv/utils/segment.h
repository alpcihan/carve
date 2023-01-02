#pragma once

#include <opencv2/opencv.hpp>

namespace crv
{
    namespace segment
    {
        void evaluateObjectOnlyImage(const cv::Mat &image, cv::Mat& masked);
    }
}