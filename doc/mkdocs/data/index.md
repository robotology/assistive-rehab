# Assistive-rehab

!!! quote ""
    Assistive-rehab is a framework for developing the assistive intelligence of [R1 robot](https://www.youtube.com/watch?v=TBphNGW6m4o) for clinical rehabilitation. The project is being developed within the Joint Lab between [IIT](https://www.iit.it) and [Fondazionce Don Carlo Gnocchi Onlus](https://www.dongnocchi.it).

## Scenario

In a typical scenario, R1 guides a group of patients in performing physical exercises for the upper limbs and the torso.
The robot physically shows to each patient the specific exercise to perform, while verifying the correct execution and providing a verbal feedback in real-time.

## Components

The demo is implemented through the following components:

![index-8fd38317](https://user-images.githubusercontent.com/9716288/51750778-63f8df00-20b3-11e9-80a3-24bb870ecc82.png)

- R1 is equipped with an [Intel RealSense D435](https://realsense.intel.com/stereo/) depth camera, which provides RGB images along with depth data;
- acquisition and storage are carried out by the ACQ and the OPC module respectively. The ACQ module merges 2D skeleton data, as acquired by [`yarpOpenPose`](https://github.com/robotology/human-sensing), along with the depth information provided by the depth sensor. 3D skeletons are stored in the OPC. This architecture also allows to [replay experiments](replay_an_experiment.md), using the combination of `yarpdatadumper` and `yarpdataplayer`, giving to physiotherapists the possibility of computing new metrics (not run online) on the replayed experiment, and of maintaining a logbook of all the patient's exercises;
- the face recognition pipeline is responsible for associating a labeled face to a skeleton, which thus becomes unambiguously tagged. This allows potentially to personalize exercises according to the specific rehabilitation path of a patient;
- the analysis of motion extracts in real-time salient metrics, as the Range of Motion (RoM) of the articular angles (for abduction and internal/external rotation), and the speed of the end-point (for a reaching task). The module also exports relevant data for enabling offline reporting of the experiments at the end of the session. The motion analysis provides a useful tool to physiotherapists for both evaluating the quality of movement of a patient and benefiting from the offline report as required documentation to produce after a session;
- the final feedback is produced by means of an action recognizer and a feedback analysis. The action recognizer uses 2D keypoints trajectories to predict the label of the performed exercise and triggers the analysis only if the predicted label is the same as the exercise to perform. If so, the analysis compares the observed skeleton with a (predefined) template skeleton to produce real-time feedback on the range of motion, the speed and how well the target is reached;
- the robot's movement is controlled through [`ctpService`](http://www.icub.org/doc/icub-main/group__ctpService.html);
- the `interactionManager` manages the interaction between all the modules involved.
