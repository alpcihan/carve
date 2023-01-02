#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <Eigen/Dense>

namespace crv
{
    class VideoReader
    {
    public:
        VideoReader(const std::string &fileName, uint32_t frameSkip = 1);
        ~VideoReader();

        bool getNextFrame(cv::Mat &frame);
        void reset();
        
        uint32_t width() { return m_dims[0]; } 
        uint32_t height() { return m_dims[1]; } 

    private:
        Eigen::Vector2i m_dims;
        uint32_t m_totalFrames = 0;
        uint32_t m_frameSkip;

        std::string m_fileName = "";

        cv::VideoCapture m_videoCapture;
        int m_nextFrameIndex = 0;
    };
}