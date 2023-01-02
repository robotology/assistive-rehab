ARG SUPERBUILD_RELEASE="v2022.05.2"
FROM ghcr.io/robotology/assistive-rehab:${SUPERBUILD_RELEASE}-basic as speech_builder

ARG PROJECTS_DIR=/projects
ARG CMAKE_GENERATOR="Unix Makefiles"
ARG BUILD_TYPE=Release
ARG CMAKE_EXTRA_OPTIONS=-j8
ARG INSTALL_DIR="/usr/local"

LABEL maintainer="valentina.vasco@iit.it, alexandre.gomespereira@iit.it, vadim.tikhanoff@iit.it" 

# Non-interactive installation mode
ENV DEBIAN_FRONTEND=noninteractive

RUN echo "fc"

# Install modules for speech specific to FDG demo
RUN cd ${PROJECTS_DIR}/assistive-rehab && \
    git fetch --all && git pull --rebase && \
    git checkout feat/fakespeech  && \
    cd build && \
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

