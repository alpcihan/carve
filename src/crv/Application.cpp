#pragma once

#include "crv/Application.h"
#include <iostream>

namespace crv
{
    void Application::init()
    {
        // TODO: read the voxel carver parameters from a config file
        crv::VoxelCarverParams params;
        params.calibVideoSource = "";
        params.sceneVideoSource = "";
        params.markerSize = 0.05;
        params.checkerBoardDims = {6, 9};
        params.calibVideoSkipBy = 24;
        params.sceneVideoSkipBy = 24;

        m_voxelCarver = std::make_unique<VoxelCarver>(params);
    }

    void Application::run()
    {
        m_voxelCarver->run();
    }
}