#include "carve/video-reader/VideoReader.h"

namespace crv
{
    VideoReader::VideoReader(const std::string &fileName, uint32_t frameSkip)
        : m_fileName(fileName), m_frameSkip(frameSkip)
    {
        m_videoCapture.open(CRV_RELATIVE(fileName));

        if (!m_videoCapture.isOpened())
        {
            CRV_ERROR("Failed to open file: " << fileName);
            return;
        }

        m_totalFrames = m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        m_dims = cv::Vec2i(m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH), m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT));
        
        CRV_INFO("File is opened: " << m_fileName);
        CRV_INFO("Total frames: " << m_totalFrames);
    }

    VideoReader::~VideoReader()
    {
        m_videoCapture.release();
    }

    bool VideoReader::getNextFrame(cv::Mat &frame)
    {
        m_videoCapture.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, m_frameIndex);
        m_videoCapture >> frame;

        if (frame.empty())
        {
            CRV_INFO("No frame left: " + m_fileName);
            return false;
        }

        CRV_INFO("Read the frame: " << m_frameIndex << "/" << m_totalFrames << " of " << m_fileName);
        m_frameIndex += m_frameSkip;
        
        return true;
    }

    void VideoReader::reset()
    {
        m_videoCapture.set(cv::VideoCaptureProperties::CAP_PROP_POS_FRAMES, 0);
        m_frameIndex = 0;
        CRV_INFO("Reset the frames: " << m_fileName);
    }
}