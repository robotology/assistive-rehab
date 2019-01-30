| Back to the [website](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/index.html) |

# The assistive-rehab project

Assistive-rehab is a framework for developing the assistive intelligence of [R1 robot](https://www.youtube.com/watch?v=TBphNGW6m4o) for clinical rehabilitation. The project is being developed within the Joint Lab between [IIT](https://www.iit.it) and [Fondazionce Don Carlo Gnocchi Onlus](https://www.dongnocchi.it).

## Library

Assistive-rehab library provides basic functionality for handling skeletons.
The library has definitions for:

- creating a skeleton as series of keypoints linked together with a predefined structure;
- defining different types of skeletons (standard or waist);
- importing/exporting a skeleton's structure from/into a yarp Property;
- normalizing and scaling a skeleton;
- transform skeleton's keypoints to the desired reference system.

Additional functionalities are also included for filtering depth images and aligning two mono or multidimensional time-series.

## Modules

Assistive-rehab modules allow the user to:

- **retrieve 3D skeletons**: given depth image from the camera and 2D skeleton data from [`yarpOpenPose`](https://github.com/robotology/human-sensing), `skeletonRetriever` produces 3D skeletons and adds them in a yarp oriented database through [`objectsPropertiesCollector`](http://www.icub.org/doc/icub-main/group__objectsPropertiesCollector.html);
- **visualize 3D skeletons**: the output of `skeletonRetriever` can be visualized in real-time on the `skeletonViewer`;
- **analyze human motion**: the quality of the movement can be evaluated in real-time through `motionAnalyzer`, by specifying the tag of the metric under analysis. Metrics as the range of motion and the speed of the end-point are currently implemented;
- **recognize human actions**: 2D skeleton's keypoints can feed the `actionRecognizer` for predicting the label of the exercise being performed;
- **produce a verbal feedback**: a feedback can be produced by `feedbackProducer` and translated to verbal through `feedbackSynthetizer`;
- **replay and manipulate a recorded skeleton**: a skeleton recorded by means of `yarpdatadumper` can be played back through `skeletonPlayer`.  

Additional details can be found in the related [Modules](https://robotology.github.io/assistive-rehab/doc/doxygen/doc/html/modules.html) section.

## Applications for the robot R1

Assistive-rehab applications are listed below:

- _AssistiveRehab.xml_ and _AssistiveRehab-faces.xml_: for running the main demo without and with the face recognition pipeline. Tutorial for these applications can be found [here](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/main_apps/);
- _skeletonDumper.xml_, _skeletonDumper-faces.xml_, _AssistiveRehab-replay.xml_: for saving data without and with faces and replaying a saved experiment. Tutorial for these applications can be found [here](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/replay_an_experiment/).

## Datasets

Datasets used to train an LSTM for the action recognition pipeline can be found [here](https://github.com/robotology/assistive-rehab-storage).
