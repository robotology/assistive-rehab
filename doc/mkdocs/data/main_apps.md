# How to run the main applications

This tutorial will show you how to run the main applications of the repository.

The main applications can be found [here](https://github.com/robotology/assistive-rehab/tree/master/app/scripts), namely _AssistiveRehab.xml.template_ and _AssistiveRehab-faces.xml.template_.

## The basic app: AssistiveRehab.xml

Let's start with the basic one: _AssistiveRehab.xml.template_.

!!! tip
    _AssistiveRehab-faces.xml.template_ builds upon _AssistiveRehab.xml.template_, introducing additional modules for face recognition.

We assume you are working on the robot R1 and that `r1-face`, `r1-torso1`, `r1-cuda-linux`, `r1-console-linux`, `r1-base` and `r1-display-linux` are available.

!!! note "R1"
    On R1, `yarpserver` runs on `r1-base`.

!!! note "R1-mk2"    
    On R1-mk2, the cuda system is named `r1-console-cuda`, which we also use as display (therefore `r1-display-linux` is `r1-console-cuda`).

The interaction requires the robot's motors to be on. Therefore turn on the motors, then open a terminal and type:

```
ssh r1-base
cd $ROBOT_CODE/cer/app/robots/CER02
yarprobotinterface
```

Now we are ready to run the application!
Open a new terminal and type:

```
yarpmanager
```

Now click on _Assistive_Rehab_App_, hit run and then connect.
The demo is now running! The robot starts looking around to engage a user. If the user accepts the invitation by lifting her/his hand, the interaction starts. The user has to repeat the exercise shown by the robot, which in turns evaluates how the exercise is being performed, through a verbal feedback. The session ends with the robot giving an overall feedback of the exercise. The interaction can go on, as the robot keeps memory of the past interaction.

Let's look a bit deeper into the application to see which are the modules running:

- `yarpdev --device speech --lingware-context speech --default-language it-IT --robot r1 --pitch 80 --speed 110`: to run the speech, by default in italian. You have to change the default-language to en-GB to switch to english;
- `yarpdev --device faceDisplayServer`: to run the face;
- `yarpdev --device --context AssistiveRehab --from realsense2.ini`: to run the camera;
- `faceExpressionImage`: to visualize the expressions and the talking mouth on the robot's display;
- `iSpeak --package speech-dev`: to make the robot speak;
- `yarpOpenPose`: to detect 2D skeletons;
- `objectsPropertiesCollector`: to store 3D skeletons within a yarp-oriented database;
- `skeletonRetriever`: to produce 3D skeletons from depth and 2D skeletons;
- `attentionManager`: to make the robot focus and follow a skeleton;
- `cer_gaze-controller`: to control the robot's gaze;
- `motionAnalyzer --from motion-repertoire-rom12.ini`: to analyze motion. This configuration file does not include the reaching exercise, which is under testing from the robot's point of view (the motion analysis is implemented);
- `skeletonPlayer`: to replay the template skeleton;
- `skeletonScaler`: to move/rescale the template skeleton;
- `feedbackProducer`: to produce a feedback depending on the exercise;
- `actionRecognizer`: to recognize the exercise being performed;
- `feedbackSynthetizer`: to generate a verbal feedback;
- `yarpview`: to visualize depth and RGB image with 2D skeletons;
- `skeletonViewer`: to visualize 3D skeletons;
- `yarpscope`: to visualize the metric on the movement in real-time;
- `interactionManager`: to supervise the interaction;
- `ctpService` (for each arm): to send commands to the robot's arms;
- `cer_reaching-solver` + `cer_reaching-controller` (for each arm): to let the robot performing a reaching task.

When you want to stop the interaction, hit stop in `yarpmanager`.
You can produce two final reports (in italian and english) of the performed exercises by opening a terminal and typing:

```
assistive-rehab-generate-report.sh
```

Two html files will be created in the folder where you run the script.

!!! note
    Producing the report might take a while, as all the files whose names equal the name of the most recent file are processed. This allows us to create a report not only of the current session, but also of the clinical evolution of the patient.   

## The basic app integrated with face recognition: AssistiveRehab-faces.xml

The _AssistiveRehab-faces.xml.template_ builds upon the basic app, introducing additional modules for face recognition, which are the following:

!!! note "R1-mk2"
    On R1-mk2, the face recognition pipeline is run on `r1-torso2`, which has to be available.

- `humanStructure`: to restrict the area of recognition around human faces;
- `caffeCoder`: to extract a representation of the cropped image;
- `linearClassifierModule`: to classify the extracted features;
- `faceRecognizer`: to label human faces.

For running the demo, click on _Assistive_Rehab_with_Faces_App_ in `yarpmanager`, hit run and then connect.

!!! note
    `caffeCoder` takes a while to load the network. Therefore, before hitting connect, wait some seconds. You can click the refresh icon and check when the `caffeCoder` ports become green.

The demo is similar to the basic one, but now the robot interacts with people using their names!

!!! tip
    If you want to train the network with your face to the face database, you can use the provided [API](https://robotology.github.io/assistive-rehab/doc/doxygen/doc/html/classrecognition__IDL.html).
