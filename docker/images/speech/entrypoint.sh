#!/bin/bash
set -e

source /usr/local/share/robotology-superbuild/setup.sh

export GOOGLE_APPLICATION_CREDENTIALS=/root/authorization/${FILE_INPUT}
gcloud auth activate-service-account --key-file=/root/authorization/${FILE_INPUT}
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:/usr/local/share/speech

# If a CMD is passed, execute it
exec "$@"