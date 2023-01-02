#include "crv/utils/segment.h"
#include <opencv2/opencv.hpp>

namespace crv
{
    namespace segment
    {
        void evaluateObjectOnlyImage(const cv::Mat &image, cv::Mat& masked)
        {
            cv::Mat mask;
            cv::inRange(image, cv::Scalar(0, 0, 30), cv::Scalar(30, 20, 255), mask);
            cv::bitwise_and(image, image, masked, mask);
        }
    }
}