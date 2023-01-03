#include "crv/voxel-carver/VoxelCarver.h"

int main()
{    
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver(crv::VoxelCarverParams());
    voxelCarver->run();

    delete voxelCarver;
    return 0;
}