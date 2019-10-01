# How to run the virtual demo

This tutorial will show you how to run the _virtual_ demo.
The virtual demo replicates the [demo](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/main_apps/) with the real R1 in a virtual environment (`Gazebo`), with the virtual version of the robot:

![virtual-r1](https://user-images.githubusercontent.com/9716288/58802764-4104ab80-860e-11e9-868f-fce8e7708d49.png)

The virtual R1 is shown on a screen with a RealSense on the top (indicated by the red arrow in the picture).
In this demo, the user in the field of view is automatically engaged and the interaction includes three phases:

  1. **observation phase**: the virtual robot welcomes the user and _shows_ the exercise to perform;
  2. **direct imitation phase**: the virtual robot performs the exercise together with the user, while _providing a verbal feedback_ on how the exercise is being performed;
  3. **occluded imitation phase**: the virtual robot keeps performing the exercise _behind a panel_ and _stops providing the verbal feedback_.

Features like facial expressions, gazing the user and a verbal feedback are also included, such that the interaction is as close as possible to the real one.

The related application can be found [here](https://github.com/robotology/assistive-rehab/tree/master/app/scripts), named _AssistiveRehab-TWM-virtual.xml.template_.

## Dependencies

After [installing](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/install/) `assistive-rehab`, you will need the following dependencies:

- [RealSense](https://github.com/IntelRealSense/librealsense): for running the RealSense;
- [cer](https://github.com/robotology/cer): for running the gaze-controller and face expressions;
- [gazebo](http://gazebosim.org/tutorials?cat=install): for running the virtual environment;
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins): for exposing YARP interfaces in `Gazebo`;
- [cer-sim](https://github.com/robotology/cer-sim): which includes the model loaded by `Gazebo` in _AssistiveRehab-TWM-virtual.xml.template_;
- [speech](https://github.com/robotology/speech): for running the `iSpeak` module.

## Requirements

The following hardware is required:

- **RealSense camera**;
- **NVIDIA graphics card**: for running `yarpOpenPose` and `actionRecognizer`.

## Run the virtual demo

To run the demo, first run `yarpserver`.
Connect the RealSense to your laptop.
Open `yarpmanager`, run the `AssistiveRehab-TWM-virtual App` and connect.
A virtual R1 appears within the simulation environment.

!!! note
    By default, `Gazebo` has the origin and the external gui visible. To remove the origin, you can click on `View` and deselect `Origin`. To remove the gui, you can click on `Window` and `Full screen`.

When the demo is launched, the `interactionManager` waits for the command `start_observation` to start the exercise session. Specifically, the following commands should be sent _in this order_ to the rpc port `/interactionManager/cmd:rpc`:

- `start_observation`: to start the observation phase, where the robot looks for a user and, when it founds her/him, starts showing the exercise to be performed;
- `start_imitation`: to start the direct imitation phase, where the robot performs the exercise together with the user, while providing her/him with verbal feedback;
- `start_occlusion`: to start the occluded imitation phase, where the robot keeps performing the exercise behind a panel and stops providing the verbal feedback;
- `stop`: to stop the interaction.

!!! tip
    A different kind of interaction is also allowed, requiring the user to raise her/his hand to start the interaction. Such interaction includes the observation and the direct imitation phases, with the virtual robot _showing_ the exercise to perform and, right after, _performing_ the exercise while _providing a verbal feedback_ (this is the virtual version of the demo [Y1M5](Y1M5.md)). In this configuration, the command `start_with_hand` starts the interaction and, when the user raises her/his hand, the two phases are run in a row, one after the other, without waiting any additional command.

The following picture shows an _example_ of interaction during the _direct_ and _occluded_ imitation phase:

![virtual-r1](https://user-images.githubusercontent.com/9716288/58812028-45868f80-8621-11e9-8c06-95df4f0a4d7e.gif)


During the _direct_ imitation phase, the robot moves while providing verbal feedback (the mouth appears on the robot's screen). During the _occluded_ imitation phase, a panel appears occluding the moving arm and the robot stops providing verbal feedback (the mouth only appears at the end of the exercise, when the robot warns the user that the exercise is over).

!!! note
    The shown interaction is just an example and the number of repetitions of the exercise is higher by default.

The parameters used for this application can be found in the context [`train-with-me`](https://github.com/robotology/assistive-rehab/tree/master/app/conf/train-with-me). Specifically, parameters that control the repetitions of the arm movements are defined in the  `interactionManager.ini` as `nrep-show` and `nrep-perform`, respectively for the observation phase (set to `8` by default) and the imitation phase, both direct and occluded (set to `16` by default). The command `start_occlusion` can be sent after the eighth  repetition of the movement within the direct phase, to have `8` repetitions for each phase.
