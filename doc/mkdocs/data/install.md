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
- [Ipopt](https://github.com/coin-or/Ipopt)

!!! note "yarp"
    - `ENABLE_yarpcar_mjpeg ON`: to allow mjpeg compression.
    - `ENABLE_yarpcar_zfp ON`: to allow zfp compression.
    - `ENABLE_yarpmod_realsense2 ON`: to enable the realsense.

!!! note "OpenCV"
    1. Download OpenCV: `git clone https://github.com/opencv/opencv.git`.
    2. Checkout the correct branch/tag: `git checkout 3.4.0`.
    3. Download the external modules: `git clone https://github.com/opencv/opencv_contrib.git`.
    4. Checkout the same branch/tag: `git checkout 3.4.0`.
    5. Configure OpenCV by filling in `OPENCV_EXTRA_MODULES_PATH` with the path to opencv_contrib/modules and then toggling on all possible modules.
    6. Compile OpenCV.

## Optional dependencies

|  Dependency  |  License  |
| --------- | --------- |
| [TensorFlowCC](https://github.com/FloopCZ/tensorflow_cc) | MIT |
| fftw3 | GPL |
| GSL | GPL |
| [matio](https://github.com/tbeu/matio) | BSD 2-Clause |
| [VTK](https://github.com/Kitware/VTK) (8.1.0 or higher) | BSD-style |
| [cer](https://github.com/robotology/cer) | GPL |

!!! note "TensorFlowCC"
    TensorFlowCC builds and installs the TensorFlow C++ API, which is released under Apache 2.0 license.

!!! note "matio"
    On `Ubuntu 18.04`, you can install the library through apt: `sudo apt install libmatio-dev`.

!!! warning
    If an optional dependency is not found, the modules depending on it are not compiled.

!!! failure
    If you want to run the full demo, also additional dependencies are required.

!!! note "report"
    For generating the offline report, you will need to install the following python libraries (you can install them through `pip install`):

      - scipy
      - numpy
      - matplotlib
      - pandas
      - glob
      - jupyter
      - plotly
      - warnings
      - collections
      - math
      - re
      - os
      - datetime
      - IPython

        !!! note "plotly"
            You need to enable jupyter extension to allow plotly to work in jupyter notebook: `pip install "notebook>=5.3" "ipywidgets>=7.2" --user`.    

## Installation

If all the dependencies are met, proceed with the following instructions:

!!! example "From sources"

    Substitute to `<install-prefix>` the absolute path where you want to install the project.

    ````tab="GNU/Linux and macOS"
    git clone https://github.com/robotology/assistive-rehab.git
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=<install-prefix>
    make
    make install
    ````

    ````tab="Windows"
    git clone https://github.com/robotology/assistive-rehab.git
    mkdir build && cd build
    cmake .. -DCMAKE_INSTALL_PREFIX=<install-prefix>
    make
    make install
    ````
