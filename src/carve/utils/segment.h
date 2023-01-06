#pragma once

#include "carve/shared.h"

namespace crv
{
    namespace segment
    {
        void evaluateObjectOnlyImage(const cv::Mat &image, cv::Mat& masked);
    }
}