#!/bin/bash
set -e

source /usr/local/share/robotology-superbuild/setup.sh

echo "[ -r /usr/share/bash-completion/bash_completion   ] && . /usr/share/bash-completion/bash_completion" >> /root/.bashrc
echo "source /usr/local/share/robotology-superbuild/setup.sh" >> /root/.bashrc

export CER_DIR=/projects/cer/build
export GAZEBO_RESOURCE_PATH=${GAZEBO_RESOURCE_PATH}:/usr/local/share/gazebo/worlds:/projects/assistive-rehab/app/gazebo/tug
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:/usr/local/share/gazebo/models:/usr/local/share/iCub/robots:/projects/cer-sim/gazebo:/projects/assistive-rehab/app/gazebo/tug
export GAZEBO_MODEL_DATABASE_URI=http://gazebosim.org/models:https://app.gazebosim.org/fuel/models
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/usr/local/lib:/usr/lib/x86_64-linux-gnu/gazebo-11/plugins:${CER_DIR}/lib:/projects/gazebo/examples/plugins/actor_collisions/build
export PATH=${PATH}:${CER_DIR}/bin
export YARP_DATA_DIRS=${YARP_DATA_DIRS}:${CER_DIR}/share/CER:/usr/local/share/navigation 

# If a CMD is passed, execute it
exec "$@"