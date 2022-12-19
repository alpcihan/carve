// Uncomment the following line if you are compiling this code in Visual Studio
// #include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
    // create a pink image
    cv::Mat image(320, 320, CV_8UC3, cv::Scalar(155, 0, 155));

    // check for failure
    if (image.empty())
    {
        cout << "Could not open or find the image" << endl;
        cin.get(); 
        return -1;
    }

    cv::String windowName = "pink color"; 
    cv::namedWindow(windowName); 

    cv::imshow(windowName, image); 
    cv::waitKey(0); 
    cv::destroyWindow(windowName); 

    return 0;
}