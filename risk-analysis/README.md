This analysis is meant to evaluate the risks of the R1 platform when performing the [TUG test](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/TUG/).
In particular, it was used to evaluate in simulation:

- the **time** it takes to trigger a stop signal if an obstacle is detected through LIDAR;
- the **time** the robot takes to completely stop when it has received the stop signal at the **fastest** and the **application** speeds;
- the **impact force** between the robot and a sensor attached to a small cube.

## How to run the analysis

For this analysis, we built a [docker image](https://hub.docker.com/r/icubteamcode/etapas-cer-gazebo) including all the necessary installation, which can be pulled as following:

```
docker pull icubteamcode/etapas-cer-gazebo:master-unstable_sources
```

### Measuring the safety times

The folder `dockerfiles` includes a compose file, which includes all the necessary services to run the simluation.
Go inside the `dockerfiles` folder, run:

```
docker-compose up
```

Run a container:

```
docker run -it --name risk_analysis_times --network host --privileged --env DISPLAY=${DISPLAY} --env XAUTHORITY=/root/.Xauthority --mount type=bind,source=${XAUTHORITY},target=/root/.Xauthority --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix --mount type=bind,source=${HOME}/.config/yarp,target=/root/.config/yarp icubteamcode/etapas-cer-gazebo:master-unstable_sources bash
```

and then (inside the container) run the following script, specifying the desired number of trials:

```
> /projects/assistive-rehab/risk-analysis/scripts
python run-analysis.py <num_trials>
```

The script sends the commands to the modules to start the simulation and produces 4 files:
- `data_experiment_obstacle_0.3.txt` and `data_experiment_obstacle_0.6.txt`, including the trial number and the measured time at application and fastest speed respectively;
- `data_experiment_stop_0.3.txt` and `data_experiment_stop_0.6.txt`, including the trial number and the measured time at application and fastest speed respectively.


### Measuring the impact force

Run a container:

```
docker run -it --name risk_analysis_forces --network host --privileged --env DISPLAY=${DISPLAY} --env XAUTHORITY=/root/.Xauthority --mount type=bind,source=${XAUTHORITY},target=/root/.Xauthority --mount type=bind,source=/tmp/.X11-unix,target=/tmp/.X11-unix --mount type=bind,source=${HOME}/.config/yarp,target=/root/.config/yarp icubteamcode/etapas-cer-gazebo:master-unstable_sources bash
```

and then (inside the container) run the following bash script:

```
> /projects/assistive-rehab/risk-analysis/scripts
./measure-forces.sh
```

The script runs 500 simulations for both application and fastest speeds and for each saves a folder with the following naming convention:

```
forces_trials_<num_trial>_<speed>
```

Each folder contains a `data.log` with the following structure: `id time Fx Fy Fz Tx Ty Tz`, where `F` and `T` represent respectively the force and the torque measured on the related axis. 

## Results

### Measuring the safety times

We implemented the following scenarios:

- the robot moves forward and receives **a stop signal from `obstacleDetector`** when an obstacle is detected: we measure the time it takes to trigger a stop signal if an obstacle is detected through LIDAR.

https://user-images.githubusercontent.com/9716288/161000520-8bc95afd-77c7-4956-819b-4bd36b4a40af.mp4

- the robot has to reach a target and **we send a stop signal**: we measure the time it takes to completely stop as difference between the time when the stop is received and the time when the wheels speeds go below a threshold
 
https://user-images.githubusercontent.com/9716288/161000527-a25b6a2e-9064-4e7a-bf9c-1136ce0c01b7.mp4

The following are the results of the analysis over 500 trials at both fastest and application speeds:

| time to send stop | time to completely stop |
|---|---|
| <img src=https://user-images.githubusercontent.com/9716288/162912608-8505ca20-0acb-4115-8936-67b1e0c6c61c.jpg width="400"> </p> |   <p align="center">  <img src=https://user-images.githubusercontent.com/9716288/162912622-6b50fac4-7878-473b-880a-cdb80fab6c6c.jpg width="400"> |

These are the extracted statistics:

|                               | app speed (0.3 m/s)             | max speed (0.6 m/s)             |
|-------------------------------|---------------------------------|---------------------------------|
| time to send stop [s]         | M = 4.8264e-04, SD = 1.3668e-04 | M = 5.0699e-04, SD = 1.6943e-04 | 
| time to completely stop [s]   | M = 1.0141, SD = 0.1624         | M = 1.0010, SD = 0.1562         | 

### Measuring the impact force

We implemented the following scenario, where the robot crashes with a small box where a [force torque sensor](https://gazebosim.org/tutorials?tut=force_torque_sensor&cat=sensors) is attached:

https://user-images.githubusercontent.com/9716288/164662479-0a90be06-573a-4b61-89f6-da92cf2e5364.mp4

The results we got over **500 trials** are shown here for **app speed = 0.3 m/s** and **max speed = 0.6 m/s**:

<img src=https://user-images.githubusercontent.com/9716288/164674209-c4abd09c-776a-445f-b7ec-a4ba27a13227.jpg width="500">


Here are the extracted statistics:

|                   | app speed (0.3 m/s) | max speed (0.6 m/s)                   |
|-------------------|---------------------|---------------------------------------|
| impact force [N]  | M = 863.5435, SD = 376.7187 | M = 1.4406e+03, SD = 508.6602 | 