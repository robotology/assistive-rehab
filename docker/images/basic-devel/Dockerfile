#docker build -t icubteamcode/fdg-basic:v2022.05.2_devel_sources .
ARG SOURCE_IMG="ubuntu:focal"
FROM $SOURCE_IMG as builder

LABEL maintainer="valentina.vasco@iit.it, alexandre.gomespereira@iit.it, vadim.tikhanoff@iit.it" 

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
# ARG INSTALL_DIR="${PROJECTS_DIR}/robotology-superbuild/build/install/share/ICUBcontrib"
ARG INSTALL_DIR="/usr/local"
ARG SUPERBUILD_RELEASE="v2022.05.2"

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
    software-properties-common \
    wget

# Install realsense and utilities
#from https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md
RUN apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE || apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE && \
    add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" -u
RUN apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Install superbuild with its dependencies
RUN mkdir ${PROJECTS_DIR} && cd ${PROJECTS_DIR} &&\
    git clone https://github.com/robotology/robotology-superbuild.git &&\
    cd robotology-superbuild &&\
    git checkout ${SUPERBUILD_RELEASE} &&\
    ./scripts/install_apt_dependencies.sh &&\
    mkdir build && cd build &&\
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DNON_INTERACTIVE_BUILD:BOOL=ON \
        -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
        -DYCM_USE_DEPRECATED:BOOL=OFF \
        -DROBOTOLOGY_USES_GAZEBO=OFF \
        # enable basic demos for speech
        -DROBOTOLOGY_ENABLE_ICUB_BASIC_DEMOS=ON \ 
        -DYCM_EP_INSTALL_DIR=${INSTALL_DIR} \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS}  -j5

# Enable depth for yarp
RUN cd ${PROJECTS_DIR}/robotology-superbuild/src/YARP && \
    cmake . \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DENABLE_yarpcar_mjpeg=ON \
        -DENABLE_yarpmod_RGBDSensorWrapper=ON \
        -DENABLE_yarpmod_RGBDSensorClient=ON \
        -DENABLE_yarppm_bottle_compression_zlib=ON \
        -DENABLE_yarppm_image_compression_ffmpeg=ON \
        -DENABLE_yarppm_depthimage_compression_zlib=ON \
        -DENABLE_yarppm_depthimage_to_mono=ON \
        -DENABLE_yarppm_depthimage_to_rgb=ON &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install ${CMAKE_EXTRA_OPTIONS}
    
# Install VTK required for skeletonViewer
RUN cd ${PROJECTS_DIR} && git clone https://github.com/Kitware/VTK.git && \
    cd VTK && git checkout v8.2.0 && mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install

# realsense2 has been moved here from YARP 3.5
# RUN . /${PROJECTS_DIR}/robotology-superbuild/build/install/share/robotology-superbuild/setup.sh && \
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/yarp-device-realsense2.git && \
    cd yarp-device-realsense2 && mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        -DENABLE_realsense2=ON \
        -DENABLE_realsense2Tracking=ON \
        -DENABLE_realsense2withIMU=ON \
        && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS}

# Install cer
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/cer && \
    cd cer && git checkout devel && \
    mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DENABLE_faceExpressionImage=ON && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} && \
    make ${CMAKE_EXTRA_OPTIONS}

# Install assistive-rehab
# required here as it is used for building robotSkeletonPublisher
ENV CER_DIR=/projects/cer/build
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/assistive-rehab.git && \
    cd assistive-rehab && \
    git checkout feat/offline-tests && \
    mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} && \
    make install

# Execute entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/entrypoint.sh" ]

CMD ["bash"]

