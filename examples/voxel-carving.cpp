#include "carve/carve.h"

int main()
{    
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver({.voxelSpaceDim = 1000, .volumeScale = 1.0f});
    
    crv::VideoReader* calibrationVideoReader = new crv::VideoReader("resources/chessboard.mp4", 24);
    crv::VideoReader* arucoSceneVideoReader = new crv::VideoReader("resources/dragon.mp4", 24);

    cv::Mat frame, binaryImage;
    cv::Mat distCoeffs;
    cv::Matx33d intrinsics;
    cv::Vec3d rVec, tVec;

    crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, {6,9}, intrinsics, distCoeffs);

    while(arucoSceneVideoReader->readNextFrame(frame))
    {     
        if(!crv::calib::estimateCamToArUcoBoard(frame, intrinsics, distCoeffs, rVec, tVec))
        {
            continue;
        }

        crv::segment::evaluateObjectOnlyImage(frame, binaryImage);

        voxelCarver->carveByBinaryImage(binaryImage, intrinsics, distCoeffs, rVec, tVec);
    }
    
    voxelCarver->saveCurrentStateAsPLY("output.ply");

    delete calibrationVideoReader;
    delete arucoSceneVideoReader;
    delete voxelCarver;

    return 0;
} 