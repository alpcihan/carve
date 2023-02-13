# Carve

**Carve** is a C++ and OpenCV based *Voxel Carving* implementation. It uses binary mask of the target object and camera parameters (intrinsic, extrinsic and distortion parameters).
It also supports camera calibration with *chessboard* and pose estimation with *ArUco Board*.

![iter-gif](https://user-images.githubusercontent.com/37274614/218523193-d25eb98a-d7ca-4b7e-b925-5fb480f4df3d.gif)

## Getting Started
<ins>**1) Downloading the repository:**</ins>

Clone the repository with `git clone --recursive https://github.com/alpcihan/carve`.

If the repository was previously cloned non-recursively, clone the required submodules with `git submodule update --init`.

<ins>**2) Building the project:**</ins>

Use cmake (**3.25.1**) with `CMakeLists.txt` in the root directory to build the project with its dependencies (e.g. OpenCV).

> **Warning**
> CMake version below 3.25.1 might cause build errors (due to the OpenCV version that uses ArUco). If such a build error happens, please make sure that the cmake command runs the specified version.


<ins>**3) Example:**</ins>

Example source file `voxel-carving.cpp` with `main` function is located under [./examples/voxel-carving.cpp](./examples/voxel-carving.cpp). In order to create the executable ensure that `example-voxel-carving` is set as the startup project and cmake option `CARVE_BUILD_EXAMPLES` is not disabled.

The generated executable creates a point cloud and ray-marched mesh under the root directory from a *chessboard* ([chessboard.mp4](./resources/chessboard.mp4)) calibration video and a scene with a target object on an *ArUco Board* ([dragon.mp4](./resources/dragon.mp4)) for the pose estimation.

## Dependencies
- CMake 3.25.1
- OpenCV 4.6.0
