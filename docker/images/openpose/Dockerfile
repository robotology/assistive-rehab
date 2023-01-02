ARG SUPERBUILD_RELEASE="v2022.05.2"
FROM ghcr.io/robotology/assistive-rehab:${SUPERBUILD_RELEASE}-basic as builder

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
ARG INSTALL_DIR="/usr/local"

LABEL maintainer="valentina.vasco@iit.it, alexandre.gomespereira@iit.it, vadim.tikhanoff@iit.it" 

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# From https://github.com/hsp-iit/tour-guide-robot/blob/ros2/docker_stuff/docker_core2/Dockerfile
# Install CUDA and cuDNN (has to be those versions)
# We check the keys for ubuntu 18 since cuda-10 is not available on ubuntu 20 
RUN wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/cuda-ubuntu1804.pin && mv cuda-ubuntu1804.pin /etc/apt/preferences.d/cuda-repository-pin-600
# The key for ubuntu 18 is broken
RUN apt-key adv --fetch-keys http://developer.download.nvidia.com/compute/cuda/repos/ubuntu1604/x86_64/3bf863cc.pub 
RUN add-apt-repository "deb https://developer.download.nvidia.com/compute/cuda/repos/ubuntu1804/x86_64/ /"
RUN apt update && apt install -y cuda-toolkit-10-0

RUN wget https://developer.download.nvidia.com/compute/redist/cudnn/v7.6.1/Ubuntu18_04-x64/libcudnn7_7.6.1.34-1+cuda10.0_amd64.deb
RUN dpkg -i libcudnn7_7.6.1.34-1+cuda10.0_amd64.deb
RUN wget https://developer.download.nvidia.com/compute/redist/cudnn/v7.6.1/Ubuntu18_04-x64/libcudnn7-dev_7.6.1.34-1+cuda10.0_amd64.deb
RUN dpkg -i libcudnn7-dev_7.6.1.34-1+cuda10.0_amd64.deb
RUN rm libcudnn7_7.6.1.34-1+cuda10.0_amd64.deb libcudnn7-dev_7.6.1.34-1+cuda10.0_amd64.deb

# GCC Compiler (because openpose requires this version)
# Add Ubuntu 20.04 repos to install gcc-7 and remove after for safety
RUN /bin/bash -c 'echo deb [arch=amd64] http://archive.ubuntu.com/ubuntu focal main universe >> /etc/apt/sources.list'
RUN apt update && sudo apt install -y gcc-7 g++-7 && sudo rm -rf /var/lib/apt/lists/*
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 7 --slave /usr/bin/g++ g++ /usr/bin/g++-7
RUN /bin/bash -c "head -n -1 /etc/apt/sources.list > /etc/apt/file.tmp; mv /etc/apt/file.tmp /etc/apt/sources.list"

# Install openpose dependencies
RUN apt update && apt install -y \
    bzip2 \
    libpng-dev \ 
    libjpeg-dev \
    libblas-dev \
    liblapack-dev \ 
    libsqlite3-dev \
    libprotobuf-dev protobuf-compiler \
    libgoogle-glog-dev \
    libhdf5-serial-dev libhdf5-dev \
    libatlas3-base libatlas-base-dev liblapacke-dev \
    libleveldb-dev libsnappy-dev \
    libboost-all-dev 

# Build openpose
RUN cd ${PROJECTS_DIR} && git clone https://github.com/CMU-Perceptual-Computing-Lab/openpose && \
    cd openpose && git checkout tags/v1.6.0 
    # git submodule update --init --recursive --remote && \

RUN cd ${PROJECTS_DIR}/openpose && mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DDOWNLOAD_HAND_MODEL=OFF \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        &&\
    make ${CMAKE_EXTRA_OPTIONS} && \
    make install 
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 9 --slave /usr/bin/g++ g++ /usr/bin/g++-9

# Install yarpOpenPose   
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/human-sensing && \
    cd human-sensing/yarpOpenPose && \
    mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install

CMD ["bash"]

