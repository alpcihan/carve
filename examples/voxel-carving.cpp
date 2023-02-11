#include "carve/carve.h"

int main()
{    
    // set the voxel carver
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver({250, 1.0});

    crv::VideoReader* calibrationVideoReader = new crv::VideoReader("resources/qolla_calib.mp4", 10);
    crv::VideoReader* arucoSceneVideoReader = new crv::VideoReader("resources/qolla.mp4", 1);

    cv::Mat frame, binaryImage;
    cv::Mat distCoeffs;
    cv::Matx33d intrinsics;
    cv::Vec3d rVec, tVec;

    //crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, {8,5}, intrinsics, distCoeffs);
    //crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, {6,9}, intrinsics, distCoeffs);
    crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, {7,4}, intrinsics, distCoeffs);

    while(arucoSceneVideoReader->getNextFrame(frame))
    {
        crv::calib::estimateCamToARBoard(frame, intrinsics, distCoeffs, rVec, tVec);
        crv::segment::evaluateObjectOnlyImage(frame, binaryImage);

        voxelCarver->carveByBinaryImage(binaryImage, intrinsics, distCoeffs, rVec, tVec);
    }
    
    voxelCarver->saveCurrentStateAsPLY();
    std::string outName = "test_mesh.off";
    voxelCarver->marche(CRV_RELATIVE(outName));




    // delete throws errors  corrupted double-linked list  Aborted (core dumped)

    //delete calibrationVideoReader;
    //delete arucoSceneVideoReader;
    //delete voxelCarver;

    return 0;
} 