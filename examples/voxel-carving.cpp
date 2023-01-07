#include "carve/carve.h"

int main()
{    
    // set the voxel carver
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver({100, 1.0});

    crv::VideoReader* calibrationVideoReader = new crv::VideoReader("resources/chessboard.mp4", 24);
    crv::VideoReader* arucoSceneVideoReader = new crv::VideoReader("resources/dragon.mp4", 24);

    cv::Mat frame, binaryImage;
    cv::Mat distCoeffs;
    cv::Matx33d intrinsics;
    cv::Vec3d rVec, tVec;

    crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, {6,9}, intrinsics, distCoeffs);

    while(arucoSceneVideoReader->getNextFrame(frame))
    {
        crv::calib::estimateCamToARBoard(frame, intrinsics, distCoeffs, rVec, tVec);
        crv::segment::evaluateObjectOnlyImage(frame, binaryImage);

        voxelCarver->carveByBinaryImage(binaryImage, intrinsics, distCoeffs, rVec, tVec);
    }
    
    voxelCarver->saveCurrentStateAsPLY();

    delete calibrationVideoReader;
    delete arucoSceneVideoReader;
    delete voxelCarver;

    return 0;
} 