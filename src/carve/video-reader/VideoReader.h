#pragma once

#include "carve/shared.h"

namespace crv
{
    class VideoReader
    {
    public:
        VideoReader(const std::string &fileName, uint32_t frameSkip = 1);
        ~VideoReader();

        bool getNextFrame(cv::Mat &frame);

        int getFrameIndex() const { return m_frameIndex; }
        std::string getFileName() const { return m_fileName; }

        void reset();
        
        uint32_t width() const { return m_dims[0]; } 
        uint32_t height() const { return m_dims[1]; } 

    private:
        cv::Vec2i m_dims;
        uint32_t m_totalFrames = 0;
        uint32_t m_frameSkip;

        std::string m_fileName = "";

        cv::VideoCapture m_videoCapture;
        int m_frameIndex = 0;
    };
}