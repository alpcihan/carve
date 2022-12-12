add_library(externals INTERFACE)

set(BUILD_TESTING 0)
add_subdirectory(eigen)

target_include_directories(externals INTERFACE
    "./eigen/Eigen/src"
)

target_link_libraries(externals INTERFACE
    eigen
)