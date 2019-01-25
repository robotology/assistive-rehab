# How to run the visual pipeline in a disembodied manner

The visual pipeline can be run also without a physical robot, i.e. with a (disembodied) _realsense_ camera .
This tutorial will show you how to do it.

!!! warning
    The visual pipeline relies on `yarpOpenPose` and `actionRecognizer`, which require an NVIDIA graphics card to be run.

First, you need to run `yarpserver`. You can open a terminal and type:

```
yarpserver
```

Connect the _realsense_ to your laptop.

!!! seealso
    If you have not done it, you will need to [install](https://github.com/IntelRealSense/librealsense) it.

!!! warning
    `yarp` has to be compiled with `ENABLE_yarpmod_realsense2 ON`.

Open `yarpmanager`, run the Assistive_Rehab_App and hit _connect_.

!!! note
    All the modules that require a robot must not be run, i.e. `faceDisplayServer`, `faceExpressionImage`, `attentionManager`, `cer_gaze-controller`, `interactionManager`, `ctpService`, `cer_reaching-solver` and `cer_reaching-controller`.

!!! tip
    You can customize the app by removing unnecessary modules and replacing the nodes in the xml with localhost. You can save the app in the folder _$HOME/.local/share/yarp_ and exploit the [shadowing mechanism](http://www.yarp.it/yarp_data_dirs.html). In this way, when you open `yarpmanager`, the app will be automatically loaded.

Once all the modules are running, you need to send commands to `motionAnalyzer`, in order to select the metric and the skeleton that you want to analyze.
Therefore you can open a terminal and type:

```
yarp rpc /motionAnalyzer/cmd

loadMetric metric_tag
selectSkel skeleton_tag
start

```

where `metric_tag` and `skeleton_tag` are respectively the tag of the metric and the tag of the skeleton under analysis (for example _ROM_0_ and _#0_).

!!! tip
    The available metrics can be listed using the command `listMetrics`.

When the command `start` is given to `motionAnalyzer`, the visual pipeline starts, i.e. the template skeleton is loaded and visualized on `skeletonViewer`, the extracted metric is visualized on `yarpscope` and the action recognition pipeline starts.

You should have the following situation:

![disembodied-632a35ac](https://user-images.githubusercontent.com/9716288/51750753-4d528800-20b3-11e9-8215-b0c5f63db9a3.png)

A verbal feedback is also provided throughout the experiment.

When you want to stop the pipeline, you need to send a stop command to `motionAnalyzer`, by typing `stop` in the previous terminal.
