cmake_minimum_required(VERSION 3.25.1)
project(carve)

set (CMAKE_CXX_STANDARD 20)
set (CRV_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/carve/utils/file.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/carve/utils/file.h")

include_directories(${CMAKE_BINARY_DIR})

# carve source
file(GLOB_RECURSE CARVE_SOURCE
    "./src/*"
)

add_library(${CMAKE_PROJECT_NAME} ${CARVE_SOURCE})

# carve includes
target_include_directories(${CMAKE_PROJECT_NAME} 
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        "./src"
)

# externals
add_subdirectory(externals)
target_link_libraries(${CMAKE_PROJECT_NAME}
    externals
)

# examples
option(CARVE_BUILD_EXAMPLES "Build the examples for carve" ON)
if(CARVE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()