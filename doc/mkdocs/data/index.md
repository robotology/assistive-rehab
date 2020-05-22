# Assistive-rehab

!!! abstract ""
    Assistive-rehab is a framework for developing the assistive intelligence of [R1 robot](https://www.youtube.com/watch?v=TBphNGW6m4o) for clinical rehabilitation and tests. The project is being developed within the Joint Lab between [IIT](https://www.iit.it) and [Fondazione Don Carlo Gnocchi Onlus](https://www.dongnocchi.it).

## Scenarios

A humanoid platform offers the potential of providing objective and quantitative measurements, while delivering repetitive therapy with constant monitoring.
Motivated by this rationale, with the experts of Fondazione Don Gnocchi, we identified two main scenarios:

- [___rehabilitation___](#rehabilitation-components): R1 guides a patient in performing physical exercises for the upper limbs and the torso.
The robot physically shows the specific exercise to perform, while verifying the correct execution and providing a verbal feedback in real-time:

![rehab](https://user-images.githubusercontent.com/9716288/82061844-85774300-96c9-11ea-8ab4-622cbde86a8b.png)

!!! abstract "The Train with Me study"
    This scenario was adopted to compare the users' engagement and movement performance when exercising with a real robot and its virtual counterpart. Results show that both the levels of engagement and the participants’ performance are higher with the real robot than with the virtual one! Check it out in our [paper](https://link.springer.com/chapter/10.1007/978-3-030-35888-4_42).

- [___clinical tests___](#clinical-tests-components): R1 explains the clinical test to the patient, by physically pointing and reaching keypoints in the environment, while verifying if the test is passed and extracting real-time metrics. The developed test is the [Timed Up and Go (TUG)](TUG.md), with the patient having to stand up from a chair, walk for 3 meters, turn, go back to the chair and sit down:

![tug](https://user-images.githubusercontent.com/9716288/82061874-8f994180-96c9-11ea-8a52-46c396d1e716.png)


## Rehabilitation: components

The rehabilitation scenario has been developed through the following components:

![framework-rehab](https://user-images.githubusercontent.com/9716288/82192272-0b76d200-98f4-11ea-91f8-57fe37eda8c3.png)

- R1 is equipped with an [Intel RealSense D435](https://realsense.intel.com/stereo/) depth camera, which provides RGB images along with depth data;
- acquisition and storage are carried out by `skeletonRetriever` and [`objectsPropertiesCollector`](http://www.icub.org/software_documentation/group__objectsPropertiesCollector.html) respectively. `skeletonRetriever` merges 2D skeleton data, as acquired by [`yarpOpenPose`](https://github.com/robotology/human-sensing), along with the depth information provided by the depth sensor. 3D skeletons are stored in the OPC. This architecture also allows to [replay experiments](replay_an_experiment.md), using the combination of `yarpdatadumper` and `yarpdataplayer`, giving to physiotherapists the possibility of computing new metrics (not run online) on the replayed experiment, and of maintaining a logbook of all the patient's exercises;
- the face recognition pipeline is responsible for associating a labeled face to a skeleton, which thus becomes unambiguously tagged. This allows potentially to personalize exercises according to the specific rehabilitation path of a patient;
- the analysis of motion extracts in real-time salient metrics, as the Range of Motion (RoM) of the articular angles (for abduction and internal/external rotation), and the speed of the end-point (for a reaching task). The module also exports relevant data for enabling offline reporting of the experiments at the end of the session. The motion analysis provides a useful tool to physiotherapists for both evaluating the quality of movement of a patient and benefiting from the offline report as required documentation to produce after a session;
- the final feedback is produced by means of an action recognizer and a feedback analysis. The action recognizer uses 2D keypoints trajectories to predict the label of the performed exercise and triggers the analysis only if the predicted label is the same as the exercise to perform. If so, the analysis compares the observed skeleton with a (predefined) template skeleton to produce real-time feedback on the range of motion, the speed and how well the target is reached;
- the robot arms' movement is controlled through [`ctpService`](http://www.icub.org/doc/icub-main/group__ctpService.html);
- the `interactionManager` manages the interaction between all the modules involved.

## Clinical tests: components

The clinical test scenario consists of the following components:

![framework-tug](https://user-images.githubusercontent.com/9716288/82192305-17fb2a80-98f4-11ea-80ec-ee71418a3ef9.png)

We re-used some of the components developed for the rehabilitation scenario, specifically the acquisition and storage and the motion analysis, which was extended in order to cover the lower limbs and to extract metrics as the step length and width, the walking speed and the number of steps.
Additionally, we developed new components in order to enrich the human-robot interaction and the robot's perception:

- a reactive navigation system, which allows the robot to navigate based on the received perceptual stimuli: the robot can reach fixed points in the environment (such as the start and the finish lines) and follow the user maintaining a fixed distance along a straight path. In such a system, the environment is supposed to be free from obstacles;
- a speech recognition and interpretation pipeline, which relies on the Google services API. The pipeline receives the sound from an external microphone and 1. retrieves the speech transcript from Google Speech cloud services and 2. analyses such transcript to retrieve the sentence structure and meaning, relying on Google Language Cloud services. Such system provides a flexible and natural interaction, given by the capability to interpret the question, rather than simply recognize it.;
- a speech trigger system based on the use of a [Mystrom wifi button](https://mystrom.ch/wifi-button/): when pressed, the button is configured to produce a http request handled by means of a `node.js`, which triggers the speech pipeline;
- a visual line detection, which detects start and finish lines composed of specific ArUco markers placed on the floor. Such lines indicate the start and the end of the path to the patient, while being fixed reference points for the robot;
- the robot points to the reference lines using its arms through [`ctpService`](http://www.icub.org/doc/icub-main/group__ctpService.html);
- the `managerTUG` manages the interaction between all the modules involved.
