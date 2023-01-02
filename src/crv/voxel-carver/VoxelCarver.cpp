#include "crv/voxel-carver/VoxelCarver.h"
#include "crv/utils/segment.h"

namespace crv
{
    VoxelCarver::VoxelCarver(const VoxelCarverParams &voxelCarverParams)
        : m_params(voxelCarverParams)
    {
    }

    void VoxelCarver::run()
    {
        _calibCamera(); // calibrate the camera from the calibration video
        m_sceneVideo = std::make_unique<crv::VideoReader>(m_params.sceneVideoSource, m_params.sceneVideoSkipBy); // set space carving scene video
        
        cv::Mat frame, masked;
        while (m_sceneVideo->getNextFrame(frame))
        {
            segment::evaluateObjectOnlyImage(frame, masked);

            calib::CamToBoardData camToBoardData;
            calib::estimateCamToARBoard(frame, m_cam, camToBoardData);
        }
    }

    void VoxelCarver::_calibCamera()
    {
        auto calibVideo = VideoReader(m_params.calibVideoSource, m_params.calibVideoSkipBy); // set the calibration video
        calib::estimateCamMatrixAndDistortion(calibVideo, m_params.checkerBoardDims, m_cam);
    }

    void VoxelCarver::_getThresholdFromTheUser()
    {
        struct HSV {
            int h0 = 0, h1 = 180, s0 = 0, s1 = 255, v0 = 0, v1 = 255;
        };

        struct WindowData {
            HSV hsv;
            cv::Mat image;
            std::string windowName = "Setup Threshold";
        };

        WindowData data;
        m_sceneVideo->getNextFrame(data.image);
        cv::cvtColor(data.image, data.image, cv::COLOR_BGR2HSV);
        
        // create the window and trackbars
        auto callback = [](int value, void* data){   
            const cv::Mat& image = (*(WindowData*)data).image;
            HSV& hsv = (*(WindowData*)data).hsv;

            cv::Mat mask, result;
            cv::inRange(image, cv::Scalar(hsv.h0, hsv.s0, hsv.v0), cv::Scalar(hsv.h1, hsv.s1, hsv.v1), mask);
            cv::bitwise_and(image, image, result, mask);
            cv::imshow((*(WindowData*)data).windowName, result);
        };

        cv::namedWindow(data.windowName, cv::WINDOW_AUTOSIZE);
        cv::createTrackbar("Hue Max", data.windowName, &data.hsv.h1, 255, callback, &data);
        cv::createTrackbar("Hue Min", data.windowName, &data.hsv.h0, 255, callback, &data);
        cv::createTrackbar("Sat Max", data.windowName, &data.hsv.s1, 255, callback, &data);
        cv::createTrackbar("Sat Min", data.windowName, &data.hsv.s0, 255, callback, &data);
        cv::createTrackbar("Vue Max", data.windowName, &data.hsv.v1, 255, callback, &data);
        cv::createTrackbar("Vue Min", data.windowName, &data.hsv.v0, 255, callback, &data);

        cv::waitKey(0);
        m_sceneVideo->reset();
    }

    void VoxelCarver::_carve()
    {

    }
}