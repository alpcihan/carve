cmake_minimum_required(VERSION 3.25.1)
project(voxel-space-carving)

set (CMAKE_CXX_STANDARD 20)
set (CRV_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/crv/utils/file.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/crv/utils/file.h")

include_directories(${CMAKE_BINARY_DIR})

# externals
add_subdirectory(externals)

# src
file(GLOB_RECURSE SOURCE
    "./src/*"
)
add_executable(${CMAKE_PROJECT_NAME} ${SOURCE})
target_include_directories(${CMAKE_PROJECT_NAME} PUBLIC
    "./src"
)

target_link_libraries(${CMAKE_PROJECT_NAME}
    externals
)