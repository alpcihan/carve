#include "VideoReader.h"
#include "crv/log.h"
#include <opencv2/core/types_c.h>

namespace crv
{
    VideoReader::VideoReader(const std::string &fileName, uint32_t frameSkip)
        : m_fileName(fileName), m_frameSkip(frameSkip)
    {
        m_videoCapture.open(fileName);

        if (!m_videoCapture.isOpened())
        {
            CRV_ERROR("Failed to open file: " << fileName);
            return;
        }

        m_totalFrames = m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_COUNT);
        m_dims = Eigen::Vector2i(m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_WIDTH), m_videoCapture.get(cv::VideoCaptureProperties::CAP_PROP_FRAME_HEIGHT));
        CRV_INFO("File is opened: " << m_fileName);
        CRV_INFO("Total frames: " << m_totalFrames);
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
            CRV_INFO("No frame left: " + m_fileName);
            return false;
        }

        CRV_INFO("Read the frame: " << m_nextFrameIndex << "/" << m_totalFrames << " of " << m_fileName);
        m_nextFrameIndex += m_frameSkip;
        
        return true;
    }
}