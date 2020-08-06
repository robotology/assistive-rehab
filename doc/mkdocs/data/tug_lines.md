# How to detect TUG start and finish lines

This tutorial will show you how to detect start and finish lines defined for the TUG on [the robot](#detecting-lines-on-the-robot) and [in `gazebo`](#detecting-lines-in-gazebo).

## Using ArUco boards

In order to identify a line, we rely on [ArUco boards](https://docs.opencv.org/master/db/da9/tutorial_aruco_board_detection.html) provided by `OpenCV`, which are sets of [markers](https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html) whose relative position is known a-priori.
`OpenCV` provides the API to easily create and detect such boards.

!!! tip
    A board compared to a set of independent markers has the advantage of providing an estimated pose usually more accurate, which can be computed even in presence of occlusions or partial views (not all markers are required to perform pose estimation).

For the TUG, we define two lines, which are ArUco boards with a single line of 6 markers, belonging to two different marker dictionaries and thus uniquely identified.

Specifically:

- the `start-line`: where the user's chair is placed;
- the `finish-line`: which the user has to cross;

    !!! info
        Check out more on markers and dictionaries [here](https://docs.opencv.org/trunk/d5/dae/tutorial_aruco_detection.html).

The lines we use can be found [here](https://github.com/robotology/assistive-rehab/tree/master/modules/lineDetector/app/conf).
Each line is composed of two images, ending with `_0` and `_1`. You will need to print them separately and then stick them together as following:

|               |                                                                                                                                                                      |
|-------------- |--------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `start-line`  | <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/89393692-7376d080-d70b-11ea-968b-0ec9a48fad9f.png"> </p>                              |
| `finish-line` | <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/89393708-7671c100-d70b-11ea-812a-5ff552f2dff7.png"> </p>                              |
|               | origin in the bottom left corner <br> `x` (red) pointing along the line's length <br> `y` (green) pointing along the width <br> `z` (blue) pointing out of the line  |


!!! hint
    This is required to have a board big enough to be robustly detected in the image, given we use a camera resolution of `320x240 px`.

## Rationale: a world made of lines, robots and skeletons

Why do we need such lines?

The [TUG](TUG.md) itself requires a finish line indicating the end of the path to cross before going back to the chair.
We decided to rely on robot's perception and visually identify such line only once before starting the exercise.
Such information feeds our [database](https://robotology.github.io/robotology-documentation/doc/html/group__objectsPropertiesCollector.html) and thus the information is kept even if / when the line is not visible.  

When the odometry starts, the robot is assumed to be in the origin, which is however a random point in the world.
In order to have a starting reference point for the odometry, we further introduced an additional line, the `start-line`: when such line is detected, the line frame becomes the
world, with `x` pointing along the line length, `y` along its width and `z` out of the line, and robot and skeletons are referred to it.

The following shows an example of our world:

<p align="center"> <img src="https://user-images.githubusercontent.com/9716288/89395227-52af7a80-d70d-11ea-8359-6d8da36014e7.png" width=550> </p>

## Detecting lines on the robot

!!! warning "Assumptions"
    We assume the robot is running, with the motors on.

    On the robot `assistive-rehab` and its dependencies are already set, with the required applications in the `yarpmanager`. However, you can check requirements and installation instructions [here](install.md).


We will need to move the robot around and for doing so we can use the joystick to control it.
Turn on the joystick, open `yarpmanager` and click `R1_joystick_mobile_base_control`. Hit run and connect.

!!! info
    The required app can be found [here](https://github.com/robotology/cer/blob/devel/app/R1navigation/scripts/joystickBaseControl.xml).

!!! tip "Alternatively"
    Open `yarpmotorgui --robot cer --parts "(mobile_base)"` and click on the yellow pause icon to idle the wheels. Now you will be able to manually move the robot around.

In the `yarpmanager`, click on [`Assistive Rehabilitation lineDetector App`](https://github.com/robotology/assistive-rehab/blob/master/modules/lineDetector/app/scripts/lineDetector.xml.template) , hit run and connect.

!!! note
    If you are already running the demo, you can skip this last step: all the required modules are running already.

    !!! info
        `navController` and `cer_gaze_controller` are required from `lineDetector` to estimate the robot position with respect to the start line.


Next step is to adjust the robot's head to have the lines in the field of view. Therefore, open a terminal and type:

```
yarpmotorgui --robot cer --parts "(head)"
```

This will open the GUI where you will be able to directly control the head of the robot.
Move the first joint (pitch) until the line is visible in `yarpview` (up to ~`20 degree`).

Now use the joystick to move the robot in front of the `start-line`.

!!! warning "Mind the order"
    The order of detection is important: since the `start-line` is used as reference frame, this has to be detected first.

Open a terminal and type:

````tab="Terminal 1"
yarp rpc /lineDetector/cmd:rpc
detect start-line
````

<p align="center"> <img src="https://user-images.githubusercontent.com/9716288/89505153-7506ce00-d7c9-11ea-8b36-30582220810f.gif" width=700> </p>

Before starting the detection, the robot is in a random position:

- when the detection starts, the estimated pose appears on the `yarp` viewer;
- when the `ack` is provided, the detection is complete: the pose is estimated and the line appears on the `skeletonViewer`, with the robot position recalculated with respect to the line.
From now on, the start line is the origin of the world and robot and skeletons positions are referred to it.  

!!! note
    The estimated pose is filtered by means of a median filter, thus the detection takes few seconds.

We need now to detect the `finish-line`. As before, move the robot in front of the line and in `Terminal 1`, type:

````tab="Terminal 1"
detect finish-line
````

!!! success
    When the command provides an `ack`, the detection is complete and the finish line appears on `skeletonViewer`.

    <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/89506865-0c6d2080-d7cc-11ea-8ad8-41b7dc5d520f.gif" width=700> </p>

!!! important
    The robot navigates the environment, reaching the desired targets with errors accepted within some tolerances (defined in `navController`). Such errors accumulate over time and after a while it might be necessary to update the robot position (the real robot position drifts from what you see on `skeletonViewer`).

    For doing so, you can physically move the robot to one of the lines and run the command `update_odometry`, specifying the line's tag and the robot orientation with respect to the `start-line`: `skeletonViewer` will show the updated position.

## Detecting lines in `gazebo`

Now we are going to see how to detect these lines in `gazebo`.

### Dependencies

After [installing](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/install/) `assistive-rehab` and its requirements, you will need the following dependencies:

- [gazebo](http://gazebosim.org/tutorials?cat=install): for running the virtual environment;
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins): for exposing `YARP` interfaces in `gazebo`;
- [cer-sim](https://github.com/robotology/cer-sim): which includes the robot model loaded by `gazebo` in `AssistiveRehab-TUG_SIM.xml.template`;
- [cer](https://github.com/robotology/cer): for running the `cer_gaze-controller`;
- [navigation](https://github.com/robotology/navigation): for running `baseControl`.

### Preparing your environment

The first step you need to take is to prepare your environment.

The folder [`assistive-rehab/app/gazebo/tug`](https://github.com/robotology/assistive-rehab/tree/master/app/gazebo/tug) includes the `sdf` models of the Aruco boards (namely `aruco_board_start` and `aruco_board_finish`) and the world including the robot and the lines (namely `tug-scenario.world`).

Following [this](http://gazebosim.org/tutorials?tut=components&cat=get_started) gazebo tutorial, you need to let `gazebo` know where _models_ and _worlds_ are located. For doing this, you will need to set the following environment variables:

- `GAZEBO_MODEL_PATH`: has to point to the folder including the models (`assistive-rehab/app/gazebo/tug`);
- `GAZEBO_RESOURCE_PATH`: has to point to the folder including the world application (`assistive-rehab/app/gazebo/tug`);

### Running the application

In `assistive-rehab/app/scripts` you will find the application [`AssistiveRehab-TUG_SIM.xml.template`](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/AssistiveRehab-TUG_SIM.xml.template) to run the TUG demo in `gazebo`.

First, you need to run `yarpserver`, opening a terminal and typing:

```
yarpserver
```

Open `yarpmanager`, run the `AssistiveRehab-TUG_SIM.xml` and hit connect.

!!! warning
    The `xml` is conceived for the complete TUG demo, therefore not all modules are required for this specific application.
    In the complete application, `lineDetector` takes the RGB input propagated from `yarpOpenPose`. For this specific application, you don't need to run `yarpOpenPose` and can simply connect `/SIM_CER_ROBOT/depthCamera/rgbImage:o` to `/lineDetector/img:i`.

You will need to run the following modules:

- `gazebo`: to run `gazebo` and load the world;
- `objectsPropertiesCollector`: to store the robot skeleton and the lines within a yarp-oriented database;
- `robotSkeletonPublisher`: to publish the skeleton representing the robot;
- `baseControl`: to control the robot's base;
- `navController`: to send commands to `baseControl`;
- `cer_gaze-controller`: to control the robot's gaze;
- `lineDetector`: to detect the lines;
- `yarpview --name /viewer/line`: to visualize the detected lines in the RGB image;
- `skeletonViewer`: to visualize the detected lines and the robot skeleton in the world.

!!! tip
    You can customize the app by removing unnecessary modules. You can save the app in your folder `$HOME/.local/share/yarp `and exploit the [shadowing mechanism](http://www.yarp.it/yarp_data_dirs.html#datafiles_shadow). In this way, when you open `yarpmanager`, your customized app will be automatically loaded.

You should now see the `gazebo` GUI with the robot and the two Aruco boards loaded, as following:

![gazebo](https://user-images.githubusercontent.com/9716288/77070435-81d97f80-69ea-11ea-9301-6978e95617c2.png)

You can see there are two Aruco boards: we identify the closer to the robot as `finish-line` and the further as `start-line`.
Let's detect these lines!
You will need to move the robot's head to have the lines in the field of view. Therefore, open a terminal and type:

```
yarpmotorgui --robot SIM_CER_ROBOT --parts "(head)"
```

As with the real robot, move the first joint (pitch) until the line is visible in `yarpview` (up to ~`20 degree`).

Since we don't have a joystick to control the robot in `gazebo`, we can rely on `navController` for moving it.
You will then need to send commands to `navController` (`Terminal 1`) for moving the robot and `lineDetector` (`Terminal 2`) for detecting the lines. Therefore, open two terminals and type:

````tab="Terminal 1"
yarp rpc /navController/rpc
````

````tab="Terminal 2"
yarp rpc /lineDetector/cmd:rpc
````

!!! note
    From now on, I will refer to `Terminal 1` for commands provided to `navController` and `Terminal 2` for commands to `lineDetector`.

Let's start to detect the `start-line`.
When the application starts, the start line is out of the robot's field of view.
Therefore you have to move the robot to a point of the world where the line is visible. For doing so, you can use the `go_to` service provided by `navController`, typing the command shown in `Terminal 1`. This will move the robot `2.5 meters` forward keeping its current orientation and you will see the robot approaching the line. When the robot stops, the line will be in the robot's field of view and you can detect it through the `detect` service provided by `lineDetector`, typing the command shown in `Terminal 2`:

!!! note
    When the application starts, the robot reference frame follows the following convention: `X` pointing forward, `Y` pointing left, `Z` pointing up.

````tab="Terminal 1 (navController)"
go_to 2.5 0.0 0.0
````

````tab="Terminal 2 (lineDetector)"
detect start-line
````

!!! tip
    `navController` and `lineDetector` provide services through `thrift`. Therefore, you can check what's required by the specific service typing `help` followed by the name of the service (for example `help go_to`).

This is what you should see:

![start-line](https://user-images.githubusercontent.com/9716288/77072524-1db8ba80-69ee-11ea-963d-8e99b87f4392.gif)

As with the real robot, when the start line is estimated it appears in the `skeletonViewer` and becomes the origin of the world: the robot position in the `skeletonViewer` is recalculated with respect to the line.

Let's now detect the `finish-line`.
The procedure is the same as before, therefore you will have to move to robot to a point of the world where the finish line is visible.

!!! warning
    Keep in mind that the robot position is now referred to the start line.
    Therefore the robot reference frame is: `X` pointing right, `Y` pointing forward, `Z` pointing up.

Again, use the `go_to` service provided by `navController`, typing the command shown in `Terminal 1`.
The robot will start approaching the line. When the robot stops, you can detect the finish line through the `detect` service provided by `lineDetector`, typing the command shown in `Terminal 2`:

````tab="Terminal 1 (navController)"
go_to 0.0 -5.5 90.0 true
````

````tab="Terminal 2 (lineDetector)"
detect finish-line
````

!!! tip
    The `true` flag in `Terminal 1` allows the robot to move backward.

This is what you should see:

![finish-line](https://user-images.githubusercontent.com/9716288/77072911-b2bbb380-69ee-11ea-9251-6c746298a284.gif)

Finally, to check that the line is estimated correctly, you can use the `go_to_line` service provided by `lineDetector`, as shown in `Terminal 2`:

````tab="Terminal 2 (lineDetector)"
go_to_line finish-line
````

You will see the robot going to the finish line origin:

![go_to-real](https://user-images.githubusercontent.com/9716288/77073155-1c3bc200-69ef-11ea-8769-7377d9d071d0.gif)
