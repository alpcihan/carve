add_library(externals INTERFACE)

option(BUILD_TESTING OFF)
option(BUILD_PERF_TESTS OFF)
option(BUILD_opencv_python_tests OFF)

add_subdirectory(opencv) # add before eigen (due to the custom target uninstall conflict)
add_subdirectory(eigen)

target_include_directories(externals INTERFACE
    # eigen
    "./eigen/Eigen/src"

    #opencv
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
    # eigen
    eigen

    # opencv
    opencv_core
    opencv_highgui
    opencv_imgproc
)