add_library(externals INTERFACE)

# add eigen
option(BUILD_TESTING OFF)
add_subdirectory(eigen)

# add opencv
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()
option(BUILD_TIFF OFF)
option(WITH_TIFF OFF)
add_subdirectory(opencv)

target_include_directories(externals INTERFACE
    "./eigen/Eigen/src"
    "./opencv/include"
    ./opencv/modules/calib3d/include
    ./opencv/modules/core/include
    ./opencv/modules/cudaarithm/include
    ./opencv/modules/cudabgsegm/include
    ./opencv/modules/cudacodec/include
    ./opencv/modules/cudafeatures2d/include
    ./opencv/modules/cudafilters/include
    ./opencv/modules/cudaimgproc/include
    ./opencv/modules/cudalegacy/include
    ./opencv/modules/cudaobjdetect/include
    ./opencv/modules/cudaoptflow/include
    ./opencv/modules/cudastereo/include
    ./opencv/modules/cudawarping/include
    ./opencv/modules/cudev/include
    ./opencv/modules/dnn/include
    ./opencv/modules/features2d/include
    ./opencv/modules/flann/include
    ./opencv/modules/highgui/include
    ./opencv/modules/imgcodecs/include
    ./opencv/modules/imgproc/include
    ./opencv/modules/ml/include
    ./opencv/modules/objdetect/include
    ./opencv/modules/photo/include
    ./opencv/modules/shape/include
    ./opencv/modules/stitching/include
    ./opencv/modules/superres/include
    ./opencv/modules/ts/include
    ./opencv/modules/video/include
    ./opencv/modules/videoio/include
    ./opencv/modules/videostab/include
    ./opencv/modules/viz/include
    ./opencv/modules/world/include
)

target_link_libraries(externals INTERFACE 
    eigen
    opencv_core
    opencv_highgui
    opencv_imgproc
)