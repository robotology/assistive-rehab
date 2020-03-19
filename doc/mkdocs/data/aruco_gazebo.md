# How to detect Aruco boards in gazebo

This tutorial will show you how to detect Aruco boards in `gazebo`.

## Dependencies

After [installing](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/install/) `assistive-rehab` and its requirements, you will need the following dependencies:

- [gazebo](http://gazebosim.org/tutorials?cat=install): for running the virtual environment;
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins): for exposing `YARP` interfaces in `gazebo`;
- [cer-sim](https://github.com/robotology/cer-sim): which includes the robot model loaded by `gazebo` in `AssistiveRehab-TUG_SIM.xml.template`;
- [cer](https://github.com/robotology/cer): for running the `cer_gaze-controller`;
- [navigation](https://github.com/robotology/navigation): for running `baseControl`.

## Preparing your environment

The first step you need to take is to prepare your environment.

The folder [`assistive-rehab/app/gazebo/tug`](https://github.com/robotology/assistive-rehab/tree/master/app/gazebo/tug) includes the `sdf` models of the Aruco boards (namely `aruco_board_start` and `aruco_board_finish`) and the world including the robot and the lines (namely `tug-scenario.world`).

Following [this](http://gazebosim.org/tutorials?tut=components&cat=get_started) gazebo tutorial, you need to let `gazebo` know where _models_ and _worlds_ are located. For doing this, you will need to set the following environment variables:

- `GAZEBO_MODEL_PATH`: has to point to the folder including the models (`assistive-rehab/app/gazebo/tug`);
- `GAZEBO_RESOURCE_PATH`: has to point to the folder including the world application (`assistive-rehab/app/gazebo/tug`);

## Running the application

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

!!! note
    `navController` and `cer_gaze_controller` are required from `lineDetector` to estimate the robot position with respect to the start line.

You should now see the `gazebo` GUI with the robot and the two Aruco boards loaded, as following:

![gazebo](https://user-images.githubusercontent.com/9716288/77070435-81d97f80-69ea-11ea-9301-6978e95617c2.png)

!!! note
    For the TUG demo we need to identify a start line and a finish line. For this reason we have the models of two boards.

You can see there are two Aruco boards: we identify the closer to the robot as `finish-line` and the further as `start-line`.
Let's detect these lines!
You will need to move the robot's head to have the lines in the field of view. Therefore, open a terminal and type:

```
yarpmotorgui --robot SIM_CER_ROBOT --parts "(head)"
```

This will open the GUI where you will be able to directly control the head of the robot.
Move the first joint (pitch) until the line is visible in `yarpview` (up to ~`20 degree`).

You will then need to send commands to `navController` (`Terminal 1`) for moving the robot and `lineDetector` (`Terminal 2`) for detecting the lines. Therefore, open two terminals and type:

````tab="Terminal 1"
yarp rpc /navController/rpc
````

````tab="Terminal 2"
yarp rpc /lineDetector/cmd:rpc
````

!!! note
    From now on, I will refer to `Terminal 1` for commands provided to `navController` and `Terminal 2` for commands to `lineDetector`.

Let's start to detect the start line.
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

When the start line is estimated (it takes a while because the estimation is filtered by means of a median filter), it will appear in the `skeletonViewer`. From now on, the start line becomes the origin of the world: as you can see the robot position in the `skeletonViewer` is recalculated with respect to the line.

Let's now detect the finish line.
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
