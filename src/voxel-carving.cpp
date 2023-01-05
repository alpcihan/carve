#include "crv/crv.h"

int main()
{    
    crv::VoxelCarver* voxelCarver = new crv::VoxelCarver(crv::VoxelCarverParams());
    
    voxelCarver->run();
    voxelCarver->saveAsPLY();

    delete voxelCarver;
    return 0;
}