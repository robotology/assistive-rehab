#!/bin/bash

COUNT=0
TIME=2.5
SLEEP=2.5
NREP_SHOW=2
NREP_PERFORM=6

#########################
#    ABDUCTION LEFT     #
#########################
abduction_left() {
    
    NREP=$1
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    while [  $COUNT -lt $NREP ]; do
        echo "ctpq time $TIME off 1 pos (70.0)" | yarp rpc /ctpservice/left_arm/rpc
        sleep $SLEEP
        echo "ctpq time $TIME off 1 pos (16.5)" | yarp rpc /ctpservice/left_arm/rpc
        sleep $SLEEP    
        COUNT=$((COUNT+1)) 
    done
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

show_abduction_left() {

    abduction_left $NREP_SHOW
}

perform_abduction_left() {

    abduction_left $NREP_PERFORM
}

#########################
#    ABDUCTION RIGHT    #
#########################
abduction_right() {

    NREP=$1
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    while [  $COUNT -lt $NREP ]; do
        echo "ctpq time $TIME off 1 pos (85.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep $SLEEP
        echo "ctpq time $TIME off 1 pos (16.5)" | yarp rpc /ctpservice/right_arm/rpc
        sleep $SLEEP    
        COUNT=$((COUNT+1)) 
    done
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

show_abduction_right() {

    abduction_right $NREP_SHOW
}

perform_abduction_right() {

    abduction_right $NREP_PERFORM
}

##################################
#    EXTERNAL ROTATION  LEFT     #
##################################
external_rotation_left() {

    NREP=$1
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    while [  $COUNT -lt $NREP ]; do
        echo "ctpq time $TIME off 0 pos (80.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
        sleep $SLEEP
        echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/left_arm/rpc
        sleep $SLEEP
        COUNT=$((COUNT+1)) 
    done
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

show_external_rotation_left() {

    external_rotation_left $NREP_SHOW
}

perform_external_rotation_left() {

    external_rotation_left $NREP_PERFORM
}

##################################
#    EXTERNAL ROTATION  RIGHT    #
##################################
external_rotation_right() {

    NREP=$1
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    while [  $COUNT -lt $NREP ]; do
        echo "ctpq time $TIME off 0 pos (80.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep $SLEEP
        echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 80.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep $SLEEP    
        COUNT=$((COUNT+1))  
    done
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

show_external_rotation_right() {

    external_rotation_right $NREP_SHOW
}

perform_external_rotation_right() {

    external_rotation_right $NREP_PERFORM
}

##################################
#    INTERNAL ROTATION  LEFT     #
##################################
internal_rotation_left() {

    NREP=$1
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
    sleep 1.0
    while [  $COUNT -lt $NREP ]; do
        echo "ctpq time $TIME off 0 pos (-60.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
        sleep $SLEEP
        echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/left_arm/rpc
        sleep $SLEEP    
        COUNT=$((COUNT+1))  
    done
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/left_arm/rpc
}

show_internal_rotation_left() {

    internal_rotation_left $NREP_SHOW

}

perform_internal_rotation_left() {

    internal_rotation_left $NREP_PERFORM
}

##################################
#    INTERNAL ROTATION  RIGHT    #
##################################
internal_rotation_right() {

    NREP=$1
    echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
    sleep 1.0
    while [  $COUNT -lt $NREP ]; do
        echo "ctpq time $TIME off 0 pos (-60.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep $SLEEP    
        echo "ctpq time $TIME off 0 pos (0.0 83.0 0.0 85.0)" | yarp rpc /ctpservice/right_arm/rpc
        sleep $SLEEP    
        COUNT=$((COUNT+1)) 
    done
    echo "ctpq time $TIME off 0 pos (1.5 16.5 0.0 15.0 0.0 0.0 0.0 0.0)" | yarp rpc /ctpservice/right_arm/rpc
}

show_internal_rotation_right() {

    internal_rotation_right $NREP_SHOW
}

perform_internal_rotation_right() {

    internal_rotation_right $NREP_PERFORM
}

$1