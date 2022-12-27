#include "crv/VoxelCarver.h"
#include "crv/VideoReader.h"
#include "crv/calib.h"

namespace crv
{
    VoxelCarver::VoxelCarver(const VoxelCarverParams &voxelCarverParams)
        : m_params(voxelCarverParams)
    {
    }

    void VoxelCarver::run()
    {
        // path of the folder containing checkerboard images
        crv::VideoReader calibVideo(m_params.calibVideoSource, m_params.calibVideoSkipBy);
        crv::VideoReader sceneVideo(m_params.sceneVideoSource, m_params.sceneVideoSkipBy);

        cv::Mat cameraMatrix, distCoeffs;
        crv::calib::estimateCameraMatrixAndDistortion(calibVideo, m_params.checkerBoardDims, cameraMatrix, distCoeffs);

        cv::Mat frame;
        while(sceneVideo.getNextFrame(frame))
        {
            crv::calib::CameraToMarkerData data;
            crv::calib::estimateCameraToMarkers(frame, cameraMatrix, distCoeffs, data, m_params.markerSize, true);

            cv::imshow("", data.image);
            cv::waitKey(0);
        }
    }
}