// Uncomment the following line if you are compiling this code in Visual Studio
// #include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/aruco.hpp>

using namespace std;

int main(int argc, char **argv)
{


    //BILEL: Commented test code
    /*
    // create a pink image
    cv::Mat image(320, 320, CV_8UC3, cv::Scalar(155, 0, 155));

    //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

    // check for failure
    if (image.empty())
    {
        cout << "Could not open or find the image" << endl;
        cin.get(); 
        return -1;
    }

    cv::String windowName = "display"; 
    cv::namedWindow(windowName); 

    cv::imshow(windowName, image); 
    cv::waitKey(0); 
    */






    // BILEL: I did not use the videoReader Wrapper. Just tested opencv's sample.
    cv::String windowName = "display"; 
    cv::namedWindow(windowName); 
    cv::VideoCapture inputVideo("../data/suzanne.mp4");
    if (! inputVideo.isOpened())
    {
        cout << "Could not open video" << endl;
        cin.get(); 
        return -1;
    }


    //////////////////////////////////////////////////////////////
    // TODO : calibration
    //readCameraParameters(filename, cameraMatrix, distCoeffs); 
    //////////////////////////////////////////////////////////////



    // for sample blender video
    // sensor center 10.125mm  18mm
    // sensor size   20.250mm  36mm
    // image resolution 1080*1920 
    // focal length  50mm
    // focal length  2666px
    // zero distortion 
    // marker set to be 50mm in size

    // hardcoded sample params
    float intrinsic[] = {2666,0,640,0,2666,960,0,0,1};
    float distortion[] = {0,0,0,0,0};
    float markerSize = 0.05; 
    cv::Mat cameraMatrix(3, 3, CV_32F, intrinsic);
    cv::Mat distCoeffs(5, 1, CV_32F, distortion);

    //opencv example aruco pose esttimation
    int frame = 0;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    while (inputVideo.grab()) {
        frame ++;
        std::cout << "Processing frame: "<< frame << std::endl;
        cv::Mat image, imageCopy;
        inputVideo.retrieve(image);
        image.copyTo(imageCopy);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(image, dictionary, corners, ids);

        if (ids.size() > 0) {
            cv::aruco::drawDetectedMarkers(imageCopy, corners, ids);
            std::vector<cv::Vec3d> rvecs, tvecs;
            cv::aruco::estimatePoseSingleMarkers(corners, markerSize, cameraMatrix, distCoeffs, rvecs, tvecs);

            // draw axis for each marker
            for(int i=0; i<ids.size(); i++)
                cv::drawFrameAxes(imageCopy, cameraMatrix, distCoeffs, rvecs[i], tvecs[i], 0.1);
        }
        cv::imshow(windowName, imageCopy);
        char key = (char) cv::waitKey(40);
        if (key == 27)
            break;
    }

    cv::destroyWindow(windowName); 

    return 0;
}