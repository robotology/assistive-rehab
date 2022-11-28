#!/bin/bash
set -e

source /usr/local/share/robotology-superbuild/setup.sh

export YARP_DATA_DIRS=${YARP_DATA_DIRS}:/usr/local/share/speech

# If a CMD is passed, execute it
exec "$@"