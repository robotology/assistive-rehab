#!/bin/bash
set -e

source /usr/local/share/robotology-superbuild/setup.sh

echo "[ -r /usr/share/bash-completion/bash_completion   ] && . /usr/share/bash-completion/bash_completion" >> /root/.bashrc
echo "source /usr/local/share/robotology-superbuild/setup.sh" >> /root/.bashrc

export CER_DIR=/projects/cer/build
export PATH=${PATH}:${CER_DIR}/bin
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${CER_DIR}/share/CER:/projects/yarp-device-realsense2/build/share/yarp

# If a CMD is passed, execute it
exec "$@"