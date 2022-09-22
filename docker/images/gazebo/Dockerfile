#docker build -t icubteamcode/fdg-gazebo:v2022.05.2_sources .
ARG SOURCE_IMG=gazebo:libgazebo11-focal
FROM $SOURCE_IMG as builder

LABEL maintainer="valentina.vasco@iit.it, alexandre.gomespereira@iit.it, vadim.tikhanoff@iit.it" 

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
ARG INSTALL_DIR="/usr/local"
#option for superbuild
ARG SUPERBUILD_RELEASE="v2022.05.2"

ENV DEBIAN_FRONTEND=noninteractive

# Dependencies
RUN mkdir /etc/bash_completion.d/ && \
    apt update && \
    apt install -y git \
    apt-utils \
    bash-completion \
    wget \
    software-properties-common

# Clone relevant repos
RUN mkdir ${PROJECTS_DIR} && cd ${PROJECTS_DIR} &&\
    git clone https://github.com/vvasco/gazebo.git &&\
    git clone https://github.com/robotology/robotology-superbuild.git &&\
    git clone https://github.com/robotology/cer.git &&\
    git clone https://github.com/robotology/cer-sim.git &&\
    git clone https://github.com/robotology/navigation.git &&\
    git clone https://github.com/robotology/assistive-rehab.git

# Install superbuild dependencies
RUN cd ${PROJECTS_DIR}/robotology-superbuild &&\
    git checkout ${SUPERBUILD_RELEASE} &&\
    ./scripts/install_apt_dependencies.sh 

# We need this gazebo fork which includes actor changes for TUG implementation
RUN cd ${PROJECTS_DIR}/gazebo &&\
    git checkout tug && \
    mkdir build && cd build && \
    cmake .. && \
    make install ${CMAKE_EXTRA_OPTIONS}

# Build superbuild
RUN cd ${PROJECTS_DIR}/robotology-superbuild &&\
    mkdir build && cd build &&\
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DNON_INTERACTIVE_BUILD:BOOL=ON \
        -DROBOTOLOGY_ENABLE_CORE:BOOL=ON \
        -DROBOTOLOGY_USES_GAZEBO=ON \
        -DYCM_EP_INSTALL_DIR=${INSTALL_DIR} \
        &&\
    cmake --build . --target update-all -- ${CMAKE_EXTRA_OPTIONS} &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS}

# Build GazeboYARPPlugins to disable network wrappers
RUN cd ${PROJECTS_DIR}/robotology-superbuild/src/GazeboYARPPlugins &&\
    cmake . \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DGAZEBO_YARP_PLUGINS_DISABLE_IMPLICIT_NETWORK_WRAPPERS:BOOL=ON \
        -DYCM_EP_INSTALL_DIR=${INSTALL_DIR} \
        &&\
   cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
   make install ${CMAKE_EXTRA_OPTIONS}

# Clone cer-sim
# ref commit: f95408abcdfd2b94221203140c81bde607adcdec
RUN cd ${PROJECTS_DIR}/cer-sim && git checkout devel

# Build CER
# ref commit: 8d794a2980d6b8def11e2c01ef5916c78f3da6f2
RUN cd ${PROJECTS_DIR}/cer && \
    git checkout devel &&\
    mkdir build && cd build &&\
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DENABLE_cermod_cerDoubleLidar=ON \
        -DENABLE_cermod_cerOdometry=ON \
        -DENABLE_cermod_tripodMotionControl=ON \
        -DENABLE_faceExpressionImage=ON \
        && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} 
    
# Install navigation
RUN cd ${PROJECTS_DIR}/navigation && git checkout 2c3834b4db013961c1dce3264e2388ed2ed6c297 &&\
    mkdir build && cd build &&\
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install 

# Install assistive-rehab
RUN cd ${PROJECTS_DIR}/assistive-rehab && \
    mkdir build && cd build &&\
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install

# Execute entrypoint
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/entrypoint.sh" ]

CMD ["bash"]
