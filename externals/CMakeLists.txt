add_library(externals INTERFACE)

option(BUILD_TESTS OFF)
option(BUILD_TESTING OFF)
option(BUILD_PERF_TESTS OFF)
option(BUILD_opencv_python_tests OFF)
set(BUILD_opencv_aruco TRUE CACHE BOOL "BUILD OpenCV ARuco")
set(OPENCV_EXTRA_MODULES_PATH ${CMAKE_CURRENT_SOURCE_DIR}/opencv_contrib/modules CACHE PATH "Path to OpenCV extra modules")

add_subdirectory(opencv) # add before eigen (due to the custom target uninstall conflict)
add_subdirectory(eigen)

target_include_directories(externals INTERFACE
    # eigen
    ./eigen/Eigen/src

    # opencv
    ./opencv/include
    ./opencv/modules/calib3d/include
    ./opencv/modules/core/include
    ./opencv/modules/dnn/include
    ./opencv/modules/features2d/include
    ./opencv/modules/flann/include
    ./opencv/modules/gapi
    ./opencv/modules/highgui/include
    ./opencv/modules/imgcodecs/include
    ./opencv/modules/imgproc/include
    ./opencv/modules/java/include
    ./opencv/modules/js/include
    ./opencv/modules/ml/include
    ./opencv/modules/objdetect/include
    ./opencv/modules/photo/include
    ./opencv/modules/python/include
    ./opencv/modules/stitching/include
    ./opencv/modules/ts/include
    ./opencv/modules/video/include
    ./opencv/modules/videoio/include
    ./opencv/modules/world/include

    # opencv extra modules
    ./opencv_contrib/modules/aruco/include
    ./opencv_contrib/modules/bgsegm/include
    ./opencv_contrib/modules/bioinspired/include
    ./opencv_contrib/modules/cnn_3dobj/include
    ./opencv_contrib/modules/cudaarithm/include
    ./opencv_contrib/modules/cudabgsegm/include
    ./opencv_contrib/modules/cudacodec/include
    ./opencv_contrib/modules/cudafeatures2d/include
    ./opencv_contrib/modules/cudafilters/include
    ./opencv_contrib/modules/cudaimgproc/include
    ./opencv_contrib/modules/cudalegacy/include
    ./opencv_contrib/modules/cudaobjdetect/include
    ./opencv_contrib/modules/cudaoptflow/include
    ./opencv_contrib/modules/cudastereo/include
    ./opencv_contrib/modules/cudawarping/include
    ./opencv_contrib/modules/cudev/include
    ./opencv_contrib/modules/cvv/include
    ./opencv_contrib/modules/datasets/include
    ./opencv_contrib/modules/dnn_objdetect/include
    ./opencv_contrib/modules/dnn_superres/include
    ./opencv_contrib/modules/dnns_easily_fooled/include
    ./opencv_contrib/modules/dpm/include
    ./opencv_contrib/modules/face/include
    ./opencv_contrib/modules/freetype/include
    ./opencv_contrib/modules/fuzzy/include
    ./opencv_contrib/modules/hdf/include
    ./opencv_contrib/modules/hfs/include
    ./opencv_contrib/modules/img_hash/include
    ./opencv_contrib/modules/matlab/include
    ./opencv_contrib/modules/optflow/include
    ./opencv_contrib/modules/ovis/include
    ./opencv_contrib/modules/phase_unwrapping/include
    ./opencv_contrib/modules/plot/include
    ./opencv_contrib/modules/quality/include
    ./opencv_contrib/modules/reg/include
    ./opencv_contrib/modules/rgbd/include
    ./opencv_contrib/modules/saliency/include
    ./opencv_contrib/modules/sfm/include
    ./opencv_contrib/modules/shape/include
    ./opencv_contrib/modules/stereo/include
    ./opencv_contrib/modules/structured_light/include
    ./opencv_contrib/modules/superres/include
    ./opencv_contrib/modules/surface_matching/include
    ./opencv_contrib/modules/text/include
    ./opencv_contrib/modules/tracking/include
    ./opencv_contrib/modules/videostab/include
    ./opencv_contrib/modules/viz/include
    ./opencv_contrib/modules/xfeatures2d/include
    ./opencv_contrib/modules/ximgproc/include
    ./opencv_contrib/modules/xobjdetect/include
    ./opencv_contrib/modules/xphoto/include
)

target_link_libraries(externals INTERFACE 
    # eigen
    eigen

    # opencv
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_calib3d
    opencv_aruco
)