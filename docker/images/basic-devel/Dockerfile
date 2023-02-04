# docker build -t ghcr.io/robotology/assistive-rehab:master-basic-devel .

ARG SUPERBUILD_RELEASE="master"
FROM ghcr.io/robotology/assistive-rehab:${SUPERBUILD_RELEASE}-basic as builder

LABEL maintainer="mattia.fussi@iit.it, ettore.landini@iit.it, marco.randazzo@iit.it, stefano.bernagozzi@iit.it" 

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j10
# ARG INSTALL_DIR="${PROJECTS_DIR}/robotology-superbuild/build/install/share/ICUBcontrib"
ARG INSTALL_DIR="/usr/local"

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive
ENV QT_X11_NO_MITSHM 1
ENV YARP_COLORED_OUTPUT=1

ENV CER_DIR=/projects/cer/build
RUN cd ${PROJECTS_DIR} &&  \
    cd assistive-rehab && \
    rm -rf build/ && \
    git checkout feat/dockerfiles && \
    git pull --rebase && mkdir build && \
    cd build && \
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

