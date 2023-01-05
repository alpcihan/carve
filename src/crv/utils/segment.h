#pragma once

#include "crv/shared.h"

namespace crv
{
    namespace segment
    {
        void evaluateObjectOnlyImage(const cv::Mat &image, cv::Mat& masked);
    }
}