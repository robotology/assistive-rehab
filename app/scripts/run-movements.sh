#!/bin/bash

TIME=2.5

wait_motion_done() {
   
   local port="$1"
   joint="$2"
   port_sim="/SIM_CER_ROBOT$port"
   port_robot="/cer$port"
   
   if yarp ping "$port_robot" > /dev/null; then
     port=$port_robot
   elif yarp ping "$port_sim" > /dev/null; then
     port=$port_sim
   else
     echo "unable to find any port to talk to"
     exit 1
   fi
   echo "talking to $port"

  timeout=10
  t0=$(date +%s)
  while true; do
    check=$(echo "get don $joint" | yarp rpc "$port" | awk '{print $4}')
    if [ "$check" -eq "1" ]; then
      echo "motion done!"
      break
    fi
    t1=$(date +%s)
    if [ $((t1-t0)) -gt "$timeout" ]; then
      echo "timeout has just expired!"
      break
    fi
    sleep 0.5
  done
}

#########################
#    ABDUCTION LEFT     #
#########################
home_abduction_left() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 35.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

startingpos_abduction_left() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

abduction_left() {

    port=/left_arm/rpc:i
    echo "onset_movement" | yarp write ... /interactionManager/trigger:i
    echo "ctpq time $TIME off 1 pos (85.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "1"
    echo "ctpq time $TIME off 1 pos (16.5)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "1"
    echo "end_movement" | yarp write ... /interactionManager/trigger:i
}

#########################
#    ABDUCTION RIGHT    #
#########################
home_abduction_right() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 35.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

startingpos_abduction_right() {

    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

abduction_right() {
    
    port=/right_arm/rpc:i
    echo "onset_movement" | yarp write ... /interactionManager/trigger:i
    echo "ctpq time $TIME off 1 pos (85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "1"
    echo "ctpq time $TIME off 1 pos (16.5)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "1"
    echo "end_movement" | yarp write ... /interactionManager/trigger:i
}

##################################
#    EXTERNAL ROTATION  LEFT     #
##################################
home_external_rotation_left() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 35.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

startingpos_external_rotation_left() {
    
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
}
external_rotation_left() {

    port=/left_arm/rpc:i
    echo "onset_movement" | yarp write ... /interactionManager/trigger:i
    echo "ctpq time $TIME off 0 pos (80.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "end_movement" | yarp write ... /interactionManager/trigger:i
}

##################################
#    EXTERNAL ROTATION  RIGHT    #
##################################
home_external_rotation_right() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 35.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

startingpos_external_rotation_right() {

    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
}

external_rotation_right() {

    port=/right_arm/rpc:i
    echo "onset_movement" | yarp write ... /interactionManager/trigger:i
    echo "ctpq time $TIME off 0 pos (80.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "end_movement" | yarp write ... /interactionManager/trigger:i
}

##################################
#    INTERNAL ROTATION  LEFT     #
##################################
home_internal_rotation_left() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 35.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

startingpos_internal_rotation_left() {

    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
}

internal_rotation_left() {
    
    port=/left_arm/rpc:i
    echo "ctpq time $TIME off 0 pos (-60.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "end_movement" | yarp write ... /interactionManager/trigger:i
}

##################################
#    INTERNAL ROTATION  RIGHT    #
##################################
home_internal_rotation_right() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 35.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

startingpos_internal_rotation_right() {

    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
}

internal_rotation_right() {

    port=/right_arm/rpc:i
    echo "ctpq time $TIME off 0 pos (-60.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    wait_motion_done "$port" "0"
    echo "end_movement" | yarp write ... /interactionManager/trigger:i
}

#########################
#    REACHING RIGHT     #
#########################
startingpos_reaching_right() {

    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

reaching_right() {

    T=2.0
    echo "set T $T" | yarp rpc /cer_reaching-controller/right/rpc
    echo "go ((parameters ((mode "full_pose+no_torso_no_heave") (torso_heave 0.1) (lower_arm_heave 0.05))) (target (0.35 -0.35 0.55 1.0 0.0 0.0 3.1415)))" | yarp rpc /cer_reaching-controller/right/rpc
    sleep 3.5
    echo "go ((parameters ((mode "full_pose+no_torso_no_heave") (torso_heave 0.1) (lower_arm_heave 0.05))) (target (0.55 -0.33 1.2 1.0 0.0 0.0 3.1415)))" | yarp rpc /cer_reaching-controller/right/rpc
    sleep 3.5
}

#########################
#     REACHING LEFT     #
#########################
startingpos_reaching_left() {

    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

reaching_left() {

    T=2.0
    echo "set T $T" | yarp rpc /cer_reaching-controller/left/rpc
    echo "go ((parameters ((mode "full_pose+no_torso_no_heave") (torso_heave 0.1) (lower_arm_heave 0.05))) (target (0.35 0.3 0.55 0.0 0.0 0.0 0.0)))" | yarp rpc /cer_reaching-controller/left/rpc
    sleep 3.5
    echo "go ((parameters ((mode "full_pose+no_torso_no_heave") (torso_heave 0.1) (lower_arm_heave 0.05))) (target (0.55 0.33 1.2 0.0 0.0 0.0 0.0)))" | yarp rpc /cer_reaching-controller/left/rpc
    sleep 3.5
}



$1
