#docker build -t icubteamcode/fdg-basic:v2022.05.2_devel_sources .
ARG SOURCE_IMG="icubteamcode/fdg-basic:v2022.05.2_sources"
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

ENV CER_DIR=/projects/cer/build
RUN cd ${PROJECTS_DIR} &&  \
    cd assistive-rehab && git pull --rebase && \
    git checkout master && \
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

