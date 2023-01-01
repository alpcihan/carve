#pragma once

#include "crv/application/Application.h"
#include <iostream>

namespace crv
{
    void Application::init()
    {
        // TODO: read the voxel carver parameters from a config file
        m_voxelCarver = std::make_unique<VoxelCarver>(crv::VoxelCarverParams());
    }

    void Application::run()
    {
        m_voxelCarver->run();
    }

    void Application::terminate()
    {
    }
}