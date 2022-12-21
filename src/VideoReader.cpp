#include "VideoReader.h"
#include <iostream>

namespace crv
{
    VideoReader::VideoReader(const std::string &fileName)
    {
        m_videoCapture.open(fileName);

        if (!m_videoCapture.isOpened())
        {
            std::cout << "Failed to open file: " << fileName << "\n";
        }
    }

    VideoReader::~VideoReader()
    {
        m_videoCapture.release();
    }

    bool VideoReader::getNextFrame(cv::Mat &frame)
    {
        m_videoCapture >> frame;

        if (frame.empty())
        {
            return false;
        }

        return true;
    }
}