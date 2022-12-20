#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>

namespace crv
{
    class VideoReader
    {
    public:
        VideoReader(const std::string &fileName);
        ~VideoReader();

        bool getNextFrame(cv::Mat &frame);

    private:
        cv::VideoCapture m_videoCapture;
    };
}