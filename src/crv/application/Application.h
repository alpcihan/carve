#pragma once

#include "crv/voxel-carver/VoxelCarver.h"
#include <memory>

namespace crv
{
    class Application
    {
    public:
        static Application &get()
        {
            static Application instance;
            return instance;
        }

        void init();
        void run();
        void terminate();

    public:
        Application(const Application &) = delete;
        Application &operator=(const Application &) = delete;
        Application(Application &&) = delete;
        Application &operator=(Application &&) = delete;

    private:
        std::unique_ptr<VoxelCarver> m_voxelCarver = nullptr;
        bool m_isInitialized = false;

    private:
        Application(){};
    };

}