#include "carve/utils/calib.h"

namespace crv
{
    namespace calib
    {
        void estimateCamMatrixAndDistortion(VideoReader &video, const cv::Vec2i &checkerBoardDims, Cam &out)
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
                    cv::cornerSubPix(gray, corner_pts, cv::Size(11, 11), cv::Size(-1, -1), criteria);
                    objpoints.push_back(objp);
                    imgpoints.push_back(corner_pts);
                }
            }

            CRV_INFO("Calibrating the camera ("<< video.getFileName() <<")... (This might take some time dependent on the total frames used.)");

            cv::Mat R, T;
            cv::calibrateCamera(objpoints, imgpoints, cv::Size(gray.cols, gray.rows), out.cameraMatrix, out.distCoeffs, R, T);
            CRV_INFO("Camera Matrix:\n"
                     << out.cameraMatrix << "\nDistortion Coefficients:\n"
                     << out.distCoeffs);

            out.optimizedCameraMatrix = cv::getOptimalNewCameraMatrix(out.cameraMatrix, out.distCoeffs, cv::Size(video.width(), video.height()), 0);
            CRV_INFO("Optimized Camera Matrix:\n"
                     << out.optimizedCameraMatrix)
        }

        bool estimateCamToARBoard(const cv::Mat &image, const Cam &cam, CamToBoardData &out)
        {
            float markersX = 5, markersY = 7, markerLength = 0.08, markerDistance = 0.01;
            cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
            cv::Ptr<cv::aruco::GridBoard> board = cv::aruco::GridBoard::create(markersX, markersY, markerLength, markerDistance, dictionary);

            cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;

            cv::aruco::detectMarkers(image, dictionary, corners, ids); // TODO: use undistorted image

            if (ids.size() <= 0)
            {
                CRV_INFO("No AR board marker detected.");
                return false;
            }

            int valid = cv::aruco::estimatePoseBoard(corners, ids, board, cam.cameraMatrix, cam.distCoeffs, out.rVec, out.tVec);

            if (!valid)
            {
                CRV_INFO("Failed to estimate pose.");
                return false;
            }

            // create the rotation matrix
            cv::Matx33d rotMatrix;
            cv::Rodrigues(out.rVec, rotMatrix);

            // move the coordinate system to the center of the AR board
            cv::Vec3d t(
                (markersX * markerLength + (markersX - 1) * markerDistance) * 0.5,
                (markersY * markerLength + (markersY - 1) * markerDistance) * 0.5,
                0);
            t = rotMatrix * t;
            out.tVec += t;

            return true;
            // draw the result
            // image.copyTo(out.image);
            // cv::aruco::drawDetectedMarkers(out.image, corners, ids);
            // cv::drawFrameAxes(out.image, cam.cameraMatrix, cam.distCoeffs, out.rVec, out.tVec, 0.1);
        }

        void calculateUndistortedImage(const cv::Mat &image, const Cam &cam, cv::Mat &out)
        {
            // Compute the maps for remapping the image
            cv::Mat map1, map2;
            cv::initUndistortRectifyMap(cam.cameraMatrix, cam.distCoeffs, cv::Mat(), cam.optimizedCameraMatrix, image.size(), CV_16SC2, map1, map2);

            // Remap the image to remove distortion
            cv::remap(image, out, map1, map2, cv::INTER_LINEAR);
        }
    }
}