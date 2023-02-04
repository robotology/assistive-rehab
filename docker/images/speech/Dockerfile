# docker build -t ghcr.io/robotology/assistive-rehab:master-speech .

ARG SOURCE_IMG="ubuntu:focal"
FROM $SOURCE_IMG as builder

LABEL maintainer="mattia.fussi@iit.it, ettore.landini@iit.it, marco.randazzo@iit.it steano.bernagozzi@iit.it" 

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j10
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

# Pre-requisites before intalling grpc
RUN apt update &&\
    apt install -y curl gnupg \
    autoconf \
    libglib2.0-dev build-essential autoconf libtool pkg-config \
    libgflags-dev \
    golang \
    libc-ares-dev \
    libssl-dev \
    # clang and LLVM C++ lib is only required for sanitizer builds
    clang libc++-dev

# Install grpc
RUN git clone -b v1.32.0 https://github.com/grpc/grpc.git &&\
    cd grpc &&\
    git submodule update --init &&\
    mkdir build &&\
    cd build &&\
    cmake .. -DgRPC_INSTALL=ON \
            -DBUILD_SHARED_LIBS=ON \
            -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
            -DgRPC_BUILD_TESTS=OFF \
            -DgRPC_SSL_PROVIDER=package \
            -DgRPC_ZLIB_PROVIDER=package &&\
    make install ${CMAKE_EXTRA_OPTIONS}

# Add the Cloud SDK distribution URI as a package source
RUN echo "deb [signed-by=/usr/share/keyrings/cloud.google.gpg] https://packages.cloud.google.com/apt cloud-sdk main" | tee -a /etc/apt/sources.list.d/google-cloud-sdk.list

RUN apt install -y apt-transport-https ca-certificates gnupg curl

# Import the google cloud public key
RUN curl https://packages.cloud.google.com/apt/doc/apt-key.gpg | apt-key --keyring /usr/share/keyrings/cloud.google.gpg add -

RUN apt update &&\
    apt install -y google-cloud-sdk

# The next line updates PATH for the Google Cloud SDK.
RUN if [ -f '/google-cloud-sdk/path.zsh.inc' ]; then . '/google-cloud-sdk/path.zsh.inc'; fi >> /root/.bashrc

# The next line enables shell command completion for gcloud.
RUN if [ -f '/google-cloud-sdk/completion.zsh.inc' ]; then . '/google-cloud-sdk/completion.zsh.inc'; fi >> /root/.bashrc

# Needed for speechInteraction
RUN git clone https://github.com/googleapis/googleapis.git &&\
    apt install -y python-dev python3-pip \
    python3-dev \
    default-jdk &&\
    cd googleapis &&\
    sed -i '36 i FLAGS+= --experimental_allow_proto3_optional' Makefile &&\
    make &&\
    rm -rf /var/lib/apt/lists/*

# Install node.js as we need it for the WiFi button
# See here for the shell https://stackoverflow.com/questions/25899912/how-to-install-nvm-in-docker/60137919#60137919
SHELL ["/bin/bash", "--login", "-i", "-c"]
ENV NVM_DIR /root/.nvm
ENV NODE_VERSION 14.17.0 
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.35.3/install.sh | bash 

# Install yarp.js as we need it for the WiFi button
RUN cd ${PROJECTS_DIR} && \
    git clone https://github.com/robotology/yarp.js.git && \
    cd yarp.js && \
    npm install && \
    ./node_modules/cmake-js/bin/cmake-js
SHELL ["/bin/bash", "--login", "-c"]
# The node-button.js script looks into this folder for yarp.js
ENV ROBOT_CODE=/projects 

#Installing sound processing related tools and audio drivers linux
RUN apt update && apt install -y \
       pulseaudio \
       portaudio19-dev \
       libportaudio2 \
       && \
    rm -rf /var/lib/apt/lists/* 

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
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} && \
    make install

# Install speech   
RUN cd ${PROJECTS_DIR}/robotology-superbuild/src/speech && \
    cmake . \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DBUILD_SVOX_SPEECH=ON \
        -DBUILD_GOOGLE_SPEECH=ON \
        -Dgoogleapis_INCLUDE_DIR=/googleapis/gens \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install ${CMAKE_EXTRA_OPTIONS}

# Install modules for speech specific to FDG demo
RUN cd ${PROJECTS_DIR}/assistive-rehab/modules/speechInteraction && \
    mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -Dgoogleapis_INCLUDE_DIR=/googleapis/gens \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        &&\
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install

# Setup environment
ENV GOOGLE_APPLICATION_CREDENTIALS=/root/authorization/${FILE_INPUT}
# ENV auth activate-service-account --key-file=/root/authorization/${FILE_INPUT}
# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/yarp

# Execute entrypoint with google credentials
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/entrypoint.sh" ]

CMD ["bash"]

