#! /bin/bash

run() {
    velocity=$1
    trial=$2

    gazebo risk-analysis-forces.world &
    echo "Awaiting gazebo start up"
    sleep 20
    
    echo "Running obstacle detection application"
    yarpmanager-console --application /projects/assistive-rehab/risk-analysis/scripts/riskAnalysis_SIM.xml --run --connect --exit --silent

    echo "Running yarpscope"
    yarp wait /wall/analog:o
    yarpscope --min -1500.0 --max 1500.0 --remote /wall/analog:o --index "(0 1 2)" --color "(Red Green Blue)" --title "Forces" --bgcolor "White" &
    
    echo "Saving data"
    yarpdatadumper --name /forces_trial_"$trial"_"$velocity" --connect /wall/analog:o &

    echo "Stopping obstacleDetector"
    yarp wait /obstacleDetector/rpc
    echo "stop" | yarp rpc /obstacleDetector/rpc

    sleep 1
    echo "Setting speed to baseControl"
    echo "set_max_lin_vel $velocity" | yarp rpc /baseControl/rpc
    
    sleep 1
    echo "Setting speed to navController"
    yarp wait /navController/rpc
    echo "set_linear_velocity $velocity" | yarp rpc /navController/rpc

    sleep 1
    echo "Asking navController to move"
    echo "go_to_dontwait 2.0 0.0 0.0" | yarp rpc /navController/rpc

    sleep 30
}

# clean up hanging up resources
clean() {
    declare -a modules=("gzclient" "gzserver" "baseControl" "navController" "obstacleDetector" "yarpscope" "yarpdatadumper" "yarpview")
    for module in ${modules[@]}; do
        killall -9 ${module}
    done
    yarp clean
}

# main
if [[ $# -eq 0 ]]; then
    declare -a allowed_velocities=(0.3 0.6)
    echo "demo is starting up..."
    for velocity in ${allowed_velocities[@]}
    do
        for trial in {1..500}
        do
            echo "running trial $trial at speed $velocity"
            run $velocity $trial            
            clean
        done
    done
elif [ "$1" == "clean" ]; then
    echo "cleaning up resources..."
    clean
    echo "...cleanup done"
else
    echo "unknown option!"
fi
