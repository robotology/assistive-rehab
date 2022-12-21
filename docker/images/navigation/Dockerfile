#docker build -t icubteamcode/fdg-navigation:v2022.05.2_sources .
ARG SOURCE_IMG="icubteamcode/fdg-basic:v2022.05.2_sources" 
FROM $SOURCE_IMG as builder

LABEL maintainer="valentina.vasco@iit.it, alexandre.gomespereira@iit.it, vadim.tikhanoff@iit.it" 

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
ARG INSTALL_DIR="/usr/local"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

# Install navigation
RUN cd ${PROJECTS_DIR} && git clone https://github.com/robotology/navigation && \
    cd navigation && mkdir build && cd build && \
    cmake .. \
        -G "$CMAKE_GENERATOR" \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_INSTALL_PREFIX=${INSTALL_DIR} \
        && \
    cmake --build . -- ${CMAKE_EXTRA_OPTIONS} &&\
    make install

# Install assistive-rehab
RUN cd ${PROJECTS_DIR}/assistive-rehab/build && \
    make install

CMD ["bash"]

