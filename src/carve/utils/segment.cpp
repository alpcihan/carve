#include "carve/utils/segment.h"

namespace crv
{
    namespace segment
    {
        void evaluateObjectOnlyImage(const cv::Mat &image, cv::Mat& masked)
        {
            cv::Mat mask;
            cv::inRange(image, cv::Scalar(0, 0, 30), cv::Scalar(30, 20, 255), mask);
            cv::bitwise_and(image, image, masked, mask);

            cv::cvtColor(masked, masked, cv::COLOR_BGR2GRAY);
        }

        static int H_L = 0, S_L = 15, V_L = 60, H_U = 40, S_U = 255, V_U = 255, ER = 3, DL = 3;

        void Threshold_HSV(int, void* frame)
        {
            cv::Mat raw = *(cv::Mat*)frame, blurred, hsv, hsvMasked, erode, dilate;
            cv::GaussianBlur(raw, blurred, cv::Size(3, 3), 0);

            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            cv::inRange(hsv, cv::Scalar(H_L, S_L, V_L), cv::Scalar(H_U, S_U, V_U), hsvMasked);
            
            cv::erode(hsvMasked, erode, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ER, ER)));
            cv::erode(erode, erode, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ER-1, ER-1)));
            cv::dilate(erode, dilate, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DL, DL)));


            cv::Mat debug;
            cv::bitwise_and(raw, raw, debug, dilate);
            cv::imshow("erode + dilate", debug);
        }
        
        void evaluateObjectOnlyImageHSV(cv::Mat& image, cv::Mat& masked)
        {
            static bool isFirstCall = true;

            if (isFirstCall)
            {
                cv::namedWindow("HSV", cv::WINDOW_AUTOSIZE);
                cv::createTrackbar("hue lower", "HSV", &H_L, 255, Threshold_HSV, &image);
                cv::createTrackbar("hue upper", "HSV", &H_U, 255, Threshold_HSV, &image);
                cv::createTrackbar("saturation lower", "HSV", &S_L, 255, Threshold_HSV, &image);
                cv::createTrackbar("saturation upper", "HSV", &S_U, 255, Threshold_HSV, &image);
                cv::createTrackbar("value lower", "HSV", &V_L, 255, Threshold_HSV, &image);
                cv::createTrackbar("value upper", "HSV", &V_U, 255, Threshold_HSV, &image);
                cv::createTrackbar("erode", "HSV", &ER, 12, Threshold_HSV, &image);
                cv::createTrackbar("dilate", "HSV", &DL, 12, Threshold_HSV, &image);

                cv::waitKey(0);
                isFirstCall = false;
            }

            cv::Mat raw = image, blurred, hsv, hsvMasked, erode, dilate;
            
            // blur
            cv::GaussianBlur(raw, blurred, cv::Size(3, 3), 0);

            // to hsv
            cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

            // threshold by hsv
            cv::inRange(raw, cv::Scalar(H_L, S_L, V_L), cv::Scalar(H_U, S_U, V_U), masked);

            // erode and dilate
            cv::erode(hsvMasked, erode, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(ER, ER)));
            cv::dilate(erode, masked, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(DL, DL)));
        }
    }
}