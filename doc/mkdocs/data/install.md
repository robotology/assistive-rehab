# Install

!!! info "Disclaimer"
    Assistive-rehab has been widely tested on `Ubuntu 16.04` and `Ubuntu 18.04`. If you face any issue either with your OS, please submit an [Issue](https://github.com/assistive-rehab/issues).

## Requirements

- Supported Operating Systems: Linux, Windows, macOS
- C++11 compiler
- CMake 3.5
- [icub-contrib-common](https://github.com/robotology/icub-contrib-common)
- [yarp](https://github.com/robotology/yarp) (3.1.100 or higher)
- [iCub](https://github.com/robotology/icub-main)
- [OpenCV](https://github.com/opencv/opencv) (3.4.0 or higher)
- [yarpOpenPose](https://github.com/robotology/human-sensing)

!!! note "yarp"
    - `ENABLE_yarpcar_depthimage ON`: to allow sending/receiving a depth image.
    - `ENABLE_yarpcar_mjpeg ON`: to allow mjpeg compression.
    - `ENABLE_yarpcar_zfp ON`: to allow zfp compression.

!!! note "OpenCV"
    1. Download OpenCV: `git clone https://github.com/opencv/opencv.git`.
    2. Checkout the correct branch/tag: `git checkout 3.4.0`.
    3. Download the external modules: `git clone https://github.com/opencv/opencv_contrib.git`.
    4. Checkout the same branch/tag: `git checkout 3.4.0`.
    5. Configure OpenCV by filling in `OPENCV_EXTRA_MODULES_PATH` with the path to opencv_contrib/modules and then toggling on all possible modules.
    6. Compile OpenCV.

## Optional dependencies

- [TensorFlowCC](https://github.com/FloopCZ/tensorflow_cc)
- fftw3
- GSL
- [matio](https://github.com/tbeu/matio)
- IPOPT
- [VTK](https://github.com/Kitware/VTK) (8.1.0 or higher)

!!! note "matio"
    On `Ubuntu 18.04`, you can install the library through apt-get: `sudo apt-get install libmatio-dev`.

!!! warning
    If an optional dependency is not found, the modules depending on it are not compiled.

!!! failure
    If you want to run the full demo, also additional dependencies are required.

## Installation

!!! warning
    The following instructions are for Unix-like systems, but they work similarly on other operating systems.

```sh
git clone https://github.com/robotology/assistive-rehab.git
mkdir build && cd build
ccmake ..
configure and generate
make
make install
```

!!! note
    The project should be now installed in iCubContrib. If you want to install it in a different folder, set `CMAKE_INSTALL_PREFIX` to the desired install directory.
