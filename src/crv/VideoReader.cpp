#include "VideoReader.h"
#include <opencv2/core/types_c.h>
#include <iostream>

namespace crv
{
    VideoReader::VideoReader(const std::string &fileName, uint32_t frameSkip)
        : m_fileName(fileName), m_frameSkip(frameSkip)
    {
        m_videoCapture.open(fileName);

        if (!m_videoCapture.isOpened())
        {
            std::cout << "Failed to open file: " << fileName << "\n";
            return;
        }

        m_totalFrames = m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        m_dims = Eigen::Vector2i(m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH), m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT));
        std::cout << "File is opened: " << m_fileName << "\n";
        std::cout << "Total frames: " << m_totalFrames << "\n\n";
    }

    VideoReader::~VideoReader()
    {
        m_videoCapture.release();
    }

    bool VideoReader::getNextFrame(cv::Mat &frame)
    {
        m_videoCapture.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, m_nextFrameIndex);

        m_videoCapture >> frame;

        if (frame.empty())
        {
            std::cout << "[INFO] No frame left: " << m_fileName << "\n";
            return false;
        }

        std::cout << "Read the frame: " << m_nextFrameIndex << "/" << m_totalFrames << " of " << m_fileName << "\n";
        m_nextFrameIndex += m_frameSkip;
        return true;
    }
}