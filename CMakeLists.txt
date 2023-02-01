cmake_minimum_required(VERSION 3.25.1)

## PROJECT

project(carve)
set (CMAKE_CXX_STANDARD 20)

## TARGET 

file(GLOB_RECURSE CARVE_SOURCE
    "./src/*"
)
add_library(${CMAKE_PROJECT_NAME} ${CARVE_SOURCE})

target_include_directories(${CMAKE_PROJECT_NAME} 
    PUBLIC
        $<INSTALL_INTERFACE:include>
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        "./src"
)

include_directories(${CMAKE_BINARY_DIR})

set (CRV_ROOT_PATH ${CMAKE_CURRENT_SOURCE_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/src/carve/utils/file.h.in" "${CMAKE_CURRENT_SOURCE_DIR}/src/carve/utils/file.h")

# third-party
add_subdirectory(externals)
target_link_libraries(${CMAKE_PROJECT_NAME}
    externals
)

## OPTIONS

option(CARVE_ENABLE_INFO_LOGS "Enable info logs" ON)
if(CARVE_ENABLE_INFO_LOGS)
    add_definitions(-DCRV_ENABLE_INFO_LOGS)
endif()

option(CARVE_ENABLE_ERROR_LOGS "Enable error logs" ON)
if(CARVE_ENABLE_ERROR_LOGS)
    add_definitions(-DCRV_ENABLE_ERROR_LOGS)
endif()

option(CARVE_BUILD_EXAMPLES "Build the examples for carve" ON)
if(CARVE_BUILD_EXAMPLES)
    add_subdirectory(examples)
endif()