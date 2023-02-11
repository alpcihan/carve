#include "carve/utils/segment.h"

namespace crv
{
    namespace segment
    {
        void evaluateObjectOnlyImage(const cv::Mat &image, cv::Mat& masked)
        {
            cv::Mat mask;
            // // rgb
            //cv::inRange(image, cv::Scalar(0, 0, 30), cv::Scalar(30, 20, 255), mask);
            cv::inRange(image, cv::Scalar(0, 0, 0), cv::Scalar(140, 255, 255), mask);
            cv::bitwise_and(image, image, masked, mask);

            // // hsv
            //cv::Mat hsv;
            //cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
            //cv::inRange(hsv, cv::Scalar(0, 40, 50), cv::Scalar(255, 255, 240), mask);
            //cv::bitwise_and(hsv, hsv, masked, mask);
            //cv::cvtColor(masked, masked, cv::COLOR_HSV2BGR);
            
            cv::cvtColor(masked, masked, cv::COLOR_BGR2GRAY);
            std::string name = "test.jpg";
            cv::imwrite(CRV_RELATIVE(name), masked);
        }
    }
}