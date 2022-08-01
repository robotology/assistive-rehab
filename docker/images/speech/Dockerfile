#docker build -t icubteamcode/fdg-speech:v2022.05.2_sources .
ARG SPEECH_SOURCE_IMG="icubteamcode/fdg-basic:v2022.05.2_sources"
FROM $SPEECH_SOURCE_IMG as speech_builder

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
ARG INSTALL_DIR="/usr/local"

LABEL maintainer="valentina.vasco@iit.it, alexandre.gomespereira@iit.it, vadim.tikhanoff@iit.it" 

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

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

RUN apt install -y apt-transport-https ca-certificates gnupg

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
# ENV GOOGLE_APPLICATION_CREDENTIALS=/root/authorization/${FILE_INPUT}
# ENV auth activate-service-account --key-file=/root/authorization/${FILE_INPUT}
# ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib/yarp

# Execute entrypoint with google credentials
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
ENTRYPOINT [ "/usr/local/bin/entrypoint.sh" ]

CMD ["bash"]

