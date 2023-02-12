#include "carve/carve.h"

void drawCube(float size, const cv::Mat& image, const cv::Vec3d& rVec, const cv::Vec3d& tVec, const cv::Matx33d& intrinsics, const cv::Mat& distCoeffs)
{
    std::vector<cv::Point3f> points3D;
    points3D.push_back(cv::Point3f(-size, -size, 0));
    points3D.push_back(cv::Point3f(size, -size, 0));
    points3D.push_back(cv::Point3f(size, size, 0));
    points3D.push_back(cv::Point3f(-size, size, 0));
    points3D.push_back(cv::Point3f(-size, -size, size*2));
    points3D.push_back(cv::Point3f(size, -size, size*2));
    points3D.push_back(cv::Point3f(size, size, size*2));
    points3D.push_back(cv::Point3f(-size, size, size*2));

    std::vector<cv::Point2f> points2D;

    
    cv::projectPoints(points3D, rVec, cv::Vec3d(-tVec[0], -tVec[1], -tVec[2]), intrinsics, distCoeffs, points2D);

    cv::Mat result = image.clone();

    cv::line(result, points2D[0], points2D[1], cv::Scalar(0, 0, 255), 2);
    cv::line(result, points2D[1], points2D[2], cv::Scalar(0, 255, 0), 2);
    cv::line(result, points2D[2], points2D[3], cv::Scalar(255, 0, 0), 2);
    cv::line(result, points2D[3], points2D[0], cv::Scalar(0, 255, 255), 2);

    cv::line(result, points2D[4], points2D[5], cv::Scalar(255, 0, 255), 2);
    cv::line(result, points2D[5], points2D[6], cv::Scalar(255, 255, 0), 2);
    cv::line(result, points2D[6], points2D[7], cv::Scalar(255, 255, 255), 2);
    cv::line(result, points2D[7], points2D[4], cv::Scalar(0, 0, 128), 2);

    cv::line(result, points2D[0], points2D[4], cv::Scalar(128, 128, 128), 2);
    cv::line(result, points2D[1], points2D[5], cv::Scalar(128, 128, 128), 2);
    cv::line(result, points2D[2], points2D[6], cv::Scalar(128, 128, 128), 2);
    cv::line(result, points2D[3], points2D[7], cv::Scalar(128, 128, 128), 2);

    cv::imshow("Cube", result);
    cv::waitKey();
}

int main()
{    
    const uint32_t dim = 100;
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver({.voxelSpaceDim = dim, .volumeScale = 0.9f});
    
    crv::VideoReader* calibrationVideoReader = new crv::VideoReader("resources/chessboard.mp4", 24);
    crv::VideoReader* arucoSceneVideoReader = new crv::VideoReader("resources/dragon.mp4", 24);

    cv::Mat frame, binaryImage;
    cv::Mat distCoeffs;
    cv::Matx33d intrinsics;
    cv::Vec3d rVec, tVec;

    crv::calib::estimateCamMatrixAndDistortion(*calibrationVideoReader, {6,9}, intrinsics, distCoeffs);

    while(arucoSceneVideoReader->readNextFrame(frame))
    {     
        if (!crv::calib::estimateCamToArUcoBoard(frame, intrinsics, distCoeffs, rVec, tVec)) 
            continue;

        //drawCube(0.2, frame, rVec, tVec, intrinsics, distCoeffs);
        //crv::segment::evaluateObjectOnlyImageHSV(frame, binaryImage);
        crv::segment::evaluateObjectOnlyImage(frame, binaryImage);

        voxelCarver->carveByBinaryImage(binaryImage, intrinsics, distCoeffs, rVec, tVec);
    }
    
    //voxelCarver->saveCurrentStateAsPLY("output.ply");

    SimpleMesh mesh;
    
    for (int x = 0; x < dim-1; x++)
    {
        for (int y = 0; y < dim-1; y++)
        {
            for (int z = 0; z < dim-1; z++)
            {
                ProcessVolumeCell(*voxelCarver, x, y, z, 0, &mesh);
            }
        }
    }

    CRV_INFO("writing mesh");
    mesh.WriteMesh("mesh.off");

    delete calibrationVideoReader;
    delete arucoSceneVideoReader;
    delete voxelCarver;

    return 0;
} 