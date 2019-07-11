# How to save and replay an experiment

This tutorial will show you how to save and replay an experiment.

## Saving an experiment

You can save an experiment by running [this application](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/AssistiveRehab-dumpers-real.xml.template), which runs several `yarpdatadumper` for saving:

1. data from the camera and `yarpOpenPose`: 2D skeleton data, depth image and RGB image with 2D skeletons.
2. robot joints.

Saved data can be found in the folder skeletonDumper.

!!! note
    You can save the data from the virtual robot by running [this application](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/AssistiveRehab-dumpers-virtual.xml.template).

## Replaying an experiment

After saving an experiment as previously explained, [this application](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/AssistiveRehab-replay.xml.template) allows you to replay it, i.e. to visualize the saved depth, the 2D skeletons, retrieve and visualize the 3D skeletons, publish and visualize the robot skeleton.

!!! tip
    The application is conceived to visualize saved data. You can run any additional module that takes as input the saved data.  

It runs the following modules:

- `objectsPropertiesCollector`: to store 3D skeletons within a yarp-oriented database;
- `skeletonRetriever`: to produce 3D skeletons from depth and 2D skeletons;
- `robotSkeletonPublisher`: to produce 3D robot skeleton from the saved joints;
- `skeletonViewer`: to visualize 3D skeletons;
- `yarpview`: to visualize depth and RGB image with 2D skeletons.

!!! tip
    If you are only interested in the visual pipeline from the camera, you can avoid running `robotSkeletonPublisher`.

To run the application, you need a `yarpserver`. Open a terminal and type:

```
yarpserver
```

You need a `yarpdataplayer` to reproduce the dataset you have previously acquired. Open a terminal and type:

```
yarpdataplayer
```

An interface appears, you can click on _File_, then _Open Directory_. Click on the folder where you have saved your data (the folder should contain subfolders with 2D skeleton data, depth image and RGB image with 2D skeletons) and press play.

Finally open `yarpmanager`, click on the _Assistive_Rehab_Replay_App_, hit run and then connect.

!!! note
    `robotSkeletonPublisher` automatically connects to the robot ports. If you saved the data from the virtual robot using [this app](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/AssistiveRehab-dumpers-virtual.xml.template), you will need to specify the parameter `--robot SIM_CER_ROBOT`.

!!! tip
    Replaying an experiment is an important feature, with the twofold objective of:

    - allowing the physiotherapist to compute new metrics on the replayed experiment not run online;
    - performing code debbuging.
