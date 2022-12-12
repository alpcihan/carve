cmake_minimum_required(VERSION 3.21)
project(voxel-space-carving)

set (CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_BINARY_DIR})

add_subdirectory(externals)
add_subdirectory(src)

target_link_libraries(${CMAKE_PROJECT_NAME}
    externals
)