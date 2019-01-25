# How to save and replay an experiment

This tutorial will show you how to save and replay an experiment.

## Saving an experiment

You can save an experiment by running [this application](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/skeletonDumper.xml.template), which runs three `yarpdatadumper` and saves the output of `yarpOpenPose`, i.e. 2D skeleton data, depth image and RGB image with 2D skeletons. Saved data can be found in the folder skeletonDumper.

## Replaying an experiment

After saving an experiment as previously explained, [this application](https://github.com/robotology/assistive-rehab/blob/master/app/scripts/AssistiveRehab-replay.xml.template) allows you to replay it, i.e. to visualize the saved depth and the 2D skeletons and retrieve and visualize the 3D skeletons.

!!! tip
    The application is conceived to visualize saved data. You can run any additional module that takes as input the saved data.  

It runs the following modules:

- `objectsPropertiesCollector`: to store 3D skeletons within a yarp-oriented database;
- `skeletonRetriever`: to produce 3D skeletons from depth and 2D skeletons;
- `skeletonViewer`: to visualize 3D skeletons;
- `yarpview`: to visualize depth and RGB image with 2D skeletons.

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
    Replaying an experiment is an important feature, with the twofold objective of:

    - allowing the physiotherapist to compute new metrics on the replayed experiment not run online;
    - performing code debbuging.
