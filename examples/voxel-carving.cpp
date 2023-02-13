#include "carve/carve.h"

// voxel space parameters
const uint32_t VOXEL_SPACE_RESOLUTION = 100;
const float VOLUME_SIZE = 0.9f;

// input video parameters
const std::string CALIBRATION_VIDEO_PATH = "resources/chessboard.mp4"; // note: relative path
const std::string SCENE_VIDEO_PATH = "resources/dragon.mp4"; // note: relative path
const cv::Vec2i CHESSBOARD_DIMENSION = {6,9};

// output file parameters
const std::string POINTCLOUD_FILE_NAME = "output.ply";
const std::string MESH_FILE_NAME = "mesh.off";

int main()
{    
    // init the voxel carver
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver({.voxelSpaceDim = VOXEL_SPACE_RESOLUTION, .volumeScale = VOLUME_SIZE});
    
    // init the calibration and target object scene videos
    crv::VideoReader* calibrationVideoReader = new crv::VideoReader(CALIBRATION_VIDEO_PATH, 24); // chessboard video
    crv::VideoReader* arucoSceneVideoReader = new crv::VideoReader(SCENE_VIDEO_PATH, 12); // target object and ArUco Board video

    cv::Mat frame, mask;
    cv::Mat distCoeffs;
    cv::Matx33d intrinsics;
    cv::Vec3d rVec, tVec;
    // estimate the camera intrinsics and distortion coefficients from the chessboard video

    crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, CHESSBOARD_DIMENSION, intrinsics, distCoeffs);

    // read target scene video

    while(arucoSceneVideoReader->readNextFrame(frame))
    {   
        // estimate the camera pose
        if (!crv::calib::estimateCamToArUcoBoard(frame, intrinsics, distCoeffs, rVec, tVec)) 
            continue;

        // get the binary mask
        crv::segment::evaluateObjectOnlyImage(frame, mask);

        // update the voxel space by using the binary mask
        voxelCarver->carveByBinaryMask(mask, intrinsics, distCoeffs, rVec, tVec);
    }
    
    // save the voxel space state as point cloud (.ply)
    voxelCarver->saveCurrentStateAsPLY(CRV_RELATIVE(POINTCLOUD_FILE_NAME));

    // calculate the mesh with marching cubes
    CRV_INFO("Creating mesh with marching cubes...");
    SimpleMesh mesh;
    // marching cubes
    for (int x = 0; x < VOXEL_SPACE_RESOLUTION - 1; x++)
    {
        for (int y = 0; y < VOXEL_SPACE_RESOLUTION - 1; y++)
        {
            for (int z = 0; z < VOXEL_SPACE_RESOLUTION - 1; z++)
            {
                crv::mc::processVolumeCell(*voxelCarver, x, y, z, 1, &mesh);
            }
        }
    }

    // save the voxel space state as ray-marched mesh (.off)
    CRV_INFO("Mesh is created.");
    CRV_INFO("Saving the mesh...");
    mesh.writeMesh(CRV_RELATIVE(MESH_FILE_NAME));

    // free the memory
    delete calibrationVideoReader;
    delete arucoSceneVideoReader;
    delete voxelCarver;

    return 0;
} 