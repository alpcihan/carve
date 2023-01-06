add_executable(example-voxel-carving voxel-carving.cpp)
target_link_libraries(example-voxel-carving 
    PRIVATE
        carve
)