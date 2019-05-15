#!/bin/bash

TIME=2.5
SLEEP=2.5

#########################
#    ABDUCTION LEFT     #
#########################
startingpos_abduction_left() {
    
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

abduction_left() {
    
    echo "ctpq time $TIME off 1 pos (70.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep $SLEEP
    echo "ctpq time $TIME off 1 pos (16.5)" | yarp rpc /ctpservice/left_arm/rpc
    sleep $SLEEP    
}

#########################
#    ABDUCTION RIGHT    #
#########################
startingpos_abduction_right() {

    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

abduction_right() {

    echo "ctpq time $TIME off 1 pos (85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep $SLEEP
    echo "ctpq time $TIME off 1 pos (16.5)" | yarp rpc /ctpservice/right_arm/rpc
    sleep $SLEEP    
}

##################################
#    EXTERNAL ROTATION  LEFT     #
##################################
startingpos_external_rotation_left() {
    
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
}
external_rotation_left() {

    echo "ctpq time $TIME off 0 pos (80.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep $SLEEP
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep $SLEEP
}

##################################
#    EXTERNAL ROTATION  RIGHT    #
##################################
startingpos_external_rotation_right() {

    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
}

external_rotation_right() {

    echo "ctpq time $TIME off 0 pos (80.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep $SLEEP
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep $SLEEP    
}

##################################
#    INTERNAL ROTATION  LEFT     #
##################################
startingpos_internal_rotation_left() {

    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
}

internal_rotation_left() {

    echo "ctpq time $TIME off 0 pos (-60.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep $SLEEP
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep $SLEEP    
}

##################################
#    INTERNAL ROTATION  RIGHT    #
##################################
startingpos_internal_rotation_right() {

    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
}

internal_rotation_right() {

    echo "ctpq time $TIME off 0 pos (-60.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep $SLEEP    
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep $SLEEP    
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
