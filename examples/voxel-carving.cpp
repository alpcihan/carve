#include "carve/carve.h"


// Voxel Space Parameters

const uint32_t VOXEL_SPACE_RESOLUTION = 128;
const float VOLUME_SIZE = 0.09f;

// Input Video Parameters

const std::string CALIBRATION_VIDEO_PATH = "resources/chessboard.mp4";
const std::string SCENE_VIDEO_PATH = "resources/dragon.mp4";
const cv::Vec2i CHESSBOARD_DIMENSION = {6,9};

// Output File Parameters

const std::string POINTCLOUD_FILE_NAME = "output.ply";
const std::string MESH_FILE_NAME = "mesh.off";

int main()
{    
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver({.voxelSpaceDim = VOXEL_SPACE_RESOLUTION, .volumeScale = VOLUME_SIZE});
    
    crv::VideoReader* calibrationVideoReader = new crv::VideoReader(CALIBRATION_VIDEO_PATH, 24);
    crv::VideoReader* arucoSceneVideoReader = new crv::VideoReader(SCENE_VIDEO_PATH, 12);

    cv::Mat frame, mask;
    cv::Matx33d cameraMatrix;
    cv::Mat distCoeffs;
    cv::Vec3d rVec, tVec;

    crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, CHESSBOARD_DIMENSION, cameraMatrix, distCoeffs);

    while(arucoSceneVideoReader->readNextFrame(frame))
    {     
        if (!crv::calib::estimateCamToArUcoBoard(frame, cameraMatrix, distCoeffs, rVec, tVec)) 
            continue;

        //crv::segment::evaluateObjectOnlyImageHSV(frame, binaryImage);
        crv::segment::evaluateObjectOnlyImage(frame, mask);

        voxelCarver->carveByBinaryImage(mask, cameraMatrix, distCoeffs, rVec, tVec);
    }
    
    voxelCarver->saveCurrentStateAsPLY(POINTCLOUD_FILE_NAME);

    CRV_INFO("Creating mesh with marching cubes...");
    SimpleMesh mesh;
    // marching cubes
    for (int x = 0; x < VOXEL_SPACE_RESOLUTION - 1; x++)
    {
        for (int y = 0; y < VOXEL_SPACE_RESOLUTION - 1; y++)
        {
            for (int z = 0; z < VOXEL_SPACE_RESOLUTION - 1; z++)
            {
                crv::mc::processVolumeCell(*voxelCarver, x, y, z, 0, &mesh);
            }
        }
    }

    CRV_INFO("Mesh is created.");
    CRV_INFO("Saving the mesh...");
    mesh.writeMesh(MESH_FILE_NAME);

    delete calibrationVideoReader;
    delete arucoSceneVideoReader;
    delete voxelCarver;

    return 0;
} 