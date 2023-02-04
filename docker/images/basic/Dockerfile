# docker build -t ghcr.io/robotology/assistive-rehab:master-basic .

ARG SOURCE_IMG="ubuntu:jammy"
FROM $SOURCE_IMG as builder

LABEL maintainer="mattia.fussi@iit.it, ettore.landini@iit.it, marco.randazzo@iit.it steano.bernagozzi@iit.it"

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
# ARG INSTALL_DIR="${PROJECTS_DIR}/robotology-superbuild/build/install/share/ICUBcontrib"
ARG INSTALL_DIR="/usr/local"
ARG SUPERBUILD_RELEASE="master"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1
ENV YARP_COLORED_OUTPUT=1

# Dependencies
RUN apt update && apt install -y \
    git \
    # motion analysis
    libmatio-dev \
    # depth compression
    zlib1g \
    # for event collection
    libjsoncpp-dev \
    net-tools \
    software-properties-common \
    wget \
    vim \
    gdb \
    nano

# Install superbuild with its dependencies
RUN mkdir ${PROJECTS_DIR} && cd ${PROJECTS_DIR} &&\
    git clone https://github.com/robotology/robotology-superbuild.git &&\
    cd robotology-superbuild &&\
    git checkout ${SUPERBUILD_RELEASE} &&\
    ./scripts/install_apt_dependencies.sh

RUN git config --global user.email "base@dockerfile.it" && \
    git config --global user.name "docker base"

RUN cd ${PROJECTS_DIR}/robotology-superbuild/ && \
    mkdir build && cd build &&\
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DNON_INTERACTIVE_BUILD:BOOL=ON \
        -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
        -DROBOTOLOGY_PROJECT_TAGS:STRING="Unstable" \
        -DYCM_USE_DEPRECATED:BOOL=OFF \
        -DROBOTOLOGY_USES_GAZEBO:BOOL=OFF \
        -DYCM_EP_ADDITIONAL_CMAKE_ARGS:STRING="-DICUBMAIN_COMPILE_SIMULATORS:BOOL=OFF" \
        -DROBOTOLOGY_ENABLE_ICUB_BASIC_DEMOS=ON \
        #-DYCM_EP_DEVEL_MODE_YCM:BOOL=ON \
        #-DYCM_EP_DEVEL_MODE_YARP:BOOL=ON \
        -DYCM_EP_INSTALL_DIR=${INSTALL_DIR} && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS}

# Enable depth for yarp
RUN cd ${PROJECTS_DIR}/robotology-superbuild/src/YCM && \
    git checkout master && \
    git pull --rebase

RUN cd ${PROJECTS_DIR}/robotology-superbuild/src/YARP && \
    git checkout master && \
    git pull --rebase && \
    cmake . \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DENABLE_yarpcar_mjpeg=ON \
        -DENABLE_yarpmod_RGBDSensorClient=ON \
        -DENABLE_yarppm_bottle_compression_zlib=ON \
        -DENABLE_yarppm_image_compression_ffmpeg=ON \
        -DENABLE_yarppm_depthimage_compression_zlib=ON \
        -DENABLE_yarppm_depthimage_to_mono=ON \
        -DENABLE_yarppm_depthimage_to_rgb=ON &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS}

RUN cd ${PROJECTS_DIR}/robotology-superbuild/ && \
    cd build && \
    make ${CMAKE_EXTRA_OPTIONS}

# Install VTK required for skeletonViewer
RUN cd ${PROJECTS_DIR} && git clone https://github.com/Kitware/VTK.git && \
    cd VTK && git checkout v9.1.0 && mkdir build
RUN cd ${PROJECTS_DIR}/VTK/build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}
RUN cd ${PROJECTS_DIR}/VTK/build && \
    make -j8 &&\
    make install

# Install cer
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/cer && \
    cd cer && git checkout devel && \
    mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DENABLE_faceExpressionImage=ON && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} && \
    make ${CMAKE_EXTRA_OPTIONS}

# ROS2
RUN apt-get update && apt-get install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null
RUN apt-get update && apt-get upgrade -y && apt-get install -y ros-humble-desktop \
    ros-humble-test-msgs \
    python3-colcon-common-extensions python3-flake8 python3-flake8-blind-except python3-flake8-builtins \
    python3-flake8-class-newline python3-flake8-comprehensions python3-flake8-deprecated python3-flake8-docstrings \
    python3-flake8-import-order python3-flake8-quotes python3-pytest python3-pytest-cov python3-pytest-repeat \
    python3-pytest-rerunfailures python3-rosdep python3-vcstool && rosdep init && rm -rf /var/lib/apt/lists/*
RUN rosdep fix-permissions && rosdep update

# YARP-DEVICES-ROS2
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology-playground/yarp-devices-ros2 -b master && \
    cd yarp-devices-ros2/ros2_interfaces_ws && /bin/bash -c "source /opt/ros/humble/setup.bash; colcon build" && \
    cd .. && /bin/bash -c "source /opt/ros/humble/setup.bash; source /home/user1/robotology/yarp-devices-ros2/ros2_interfaces_ws/install/local_setup.bash; cmake -S. -Bbuild; cmake --build build"

# Install assistive-rehab
# required here as it is used for building robotSkeletonPublisher
ENV CER_DIR=/projects/cer/build
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/assistive-rehab.git && \
    cd assistive-rehab && \
    mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        -DUSE_SIMULATION:BOOL=OFF \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} && \
    make install

# Execute entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/entrypoint.sh" ]

CMD ["bash"]

