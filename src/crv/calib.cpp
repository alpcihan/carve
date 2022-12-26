#include "crv/calib.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types_c.h>
#include <stdio.h>
#include <iostream>

namespace crv
{
    namespace calib
    {
        void estimateCameraMatrixAndDistortion(VideoReader &video, const cv::Vec2i &checkerBoardDims, cv::Mat &cameraMatrix, cv::Mat &distCoeffs)
        {
            std::vector<std::vector<cv::Point3f>> objpoints; // vector to store vectors of 3D points for each checkerboard image
            std::vector<std::vector<cv::Point2f>> imgpoints; // vector to store vectors of 2D points for each checkerboard image
            std::vector<cv::Point2f> corner_pts;             // vector to store the pixel coordinates of detected checker board corners

            // define the world coordinates for 3D points
            std::vector<cv::Point3f> objp;
            for (int i{0}; i < checkerBoardDims[1]; i++)
            {
                for (int j{0}; j < checkerBoardDims[0]; j++)
                {
                    objp.push_back(cv::Point3f(j, i, 0));
                }
            }

            // get the object and image points of each frame
            cv::Mat frame, gray;
            while (video.getNextFrame(frame))
            {
                cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

                if (cv::findChessboardCorners(gray, cv::Size(checkerBoardDims[0], checkerBoardDims[1]), corner_pts, 1 | 8 | 2))
                {
                    cv::TermCriteria criteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.001);

                    // refine pixel coordinates for given 2d points.
                    cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);

                    // display the detected corner points on the checker board
                    // cv::drawChessboardCorners(frame, cv::Size(CHECKERBOARD[0], CHECKERBOARD[1]), corner_pts, true);

                    objpoints.push_back(objp);
                    imgpoints.push_back(corner_pts);
                }
            }

            std::cout << "\nCalibrating the camera... (This might take some time dependent on the total frames used.)\n\n";

            cv::Mat R, T;
            cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.rows, gray.cols), cameraMatrix, distCoeffs, R, T);

            std::cout << "Camera Matrix:\n" << cameraMatrix << "\nDistortion Coefficients:\n" << distCoeffs << "\n\n";

            // cameraMatrix = cv::getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, cv::Size(video.width(), video.height()), 0);
        }

        bool estimateCameraToMarkers(const cv::Mat &image, const cv::Mat &cameraMatrix, const cv::Mat &distCoeffs, CameraToMarkerData &data, float markerSize, bool storeImage)
        {
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(image, dictionary, corners, ids);

            if (ids.size() <= 0)
            {
                return false;
            }

            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);

            if (storeImage)
            {
                image.copyTo(data.image);
            }
            
            // create transformation matrices
            for (int idx = 0; idx < ids.size(); idx++)
            {
                cv::Matx33d rot_mat;
                cv::Rodrigues(rvecs[idx], rot_mat);

                // create the transformation matrix by concatenating the rotation and translation
                cv::Matx44d transform = cv::Matx44d::eye(); // 4x4 identity matrix
                for (int i = 0; i < 3; i++)
                {
                    for (int j = 0; j < 3; j++)
                    {
                        transform(i, j) = rot_mat(i, j);
                    }
                    transform(i, 3) = tvecs[idx][i];
                }

                // fill the data
                CameraToMarkerData::Data d;
                d.transform = std::move(transform);
                d.markerId = ids[idx];
                data.data.push_back(std::move(d));

                // draw the result
                if (storeImage)
                {
                    cv::drawFrameAxes(data.image, cameraMatrix, distCoeffs, rvecs[idx], tvecs[idx], 0.1);
                }
            }

            return true;
        }
    }
}