# Comparison between [Y1Q3](Y1Q3.md) and [Y1M5](Y1M5.md)

## Acquisition

The improvements that Y1Q3 introduces in terms of acquisition are listed below:

1. [Filtering the disparity map](#filtering-the-disparity-map);
2. [Optimizing the skeleton](#optimizing-the-skeleton);
3. [Reducing latencies](#reducing-latencies).

### Filtering the disparity map

The disparity map provided by the camera is inaccurate around the human contour and might have holes, leading keypoints close to the contour (e.g. hands) or on an hole to be projected incorrectly. For example, the following video shows the depth map for an abduction movement, where the hand is projected to infinite:

<p align="center">
<img src="https://user-images.githubusercontent.com/9716288/49949881-063f7780-fef7-11e8-935e-43c4d8cf9dab.gif" width="500">
</p>

In Y1M5, the disparity map is used as provided by the camera, without any additional processing, and thus it is affected by the effect described. In Y1Q3, the disparity map is eroded (and thus the depth map dilated), which has the double effect of:

1. increasing the probability for keypoints closer to the contour to fall within the correct depth map;
2. filling holes in the depth.

The following video shows the effect of filtering the depth map and keypoints falling within the correct depth:

| Unprocessed depth (Y1M5)  | Filtered depth (Y1Q3) | Keypoints inside the depth |
| ------------- | ------------- |  ------------- |
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/50230577-31b9da80-03ad-11e9-9460-53fbf7bcef76.gif width="250"> </p> |   <p align="center">  <img src=https://user-images.githubusercontent.com/9716288/50230653-5dd55b80-03ad-11e9-84db-b0b8693659fd.gif width="250"> </p> |  <p align="center">  <img src=https://user-images.githubusercontent.com/9716288/50230688-72195880-03ad-11e9-87ed-3a981c12cbee.gif width="250"> </p> |

### Optimizing the skeleton

Exercises that require movements parallel to the optical axis make some keypoints ambiguous. For example, in the external and internal rotation, the elbow is not directly observable as the hand occludes it. Therefore both keypoints are projected to the same depth, as shown here:

| Without optimization (Y1M5) |
| ------------- |
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/49950004-3a1a9d00-fef7-11e8-8657-98e1802c2dde.gif width="750"> </p> |

In Y1Q3, we introduce an optimization of the skeleton, which adjusts the depth of the keypoints such that the length of the arms is equal to that observed during an initial phase. The following image shows the result of the optimization, in which elbow and hand are projected correctly.

| With optimization (Y1Q3) |
| ------------- |
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/49950012-4272d800-fef7-11e8-8bbe-8ff08a453f1d.gif  width="750"> </p> |

### Reducing latencies

`yarpOpenPose` introduces a visible latency in the depth map, as shown here:

| Without `yarpOpenPose`  | With `yarpOpenPose` |
| ------------- | ------------- |  
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/49950278-ca58e200-fef7-11e8-88b1-0256162843f8.gif width="350" height=332> </p> |   <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/49950123-7e0da200-fef7-11e8-8a4a-e14eb6477cd3.gif  width="340"> </p> |  

In Y1Q3, `yarpOpenPose` propagates the depth image in sync with the output of skeleton detection, in order to equalize the delay between the two streams.

## Extending the feedback

### Y1M5

In Y1M5, the feedback is based on the definition of _dynamic_ joints (i.e. joints performing the exercise (1)) and _static_ joints (i.e. joints staying in the initial position (2)):

1. dynamic joints contribute to the *dynamic* score, which computes their range of motion in a predefined temporal window, by awarding joints moving within the correct range of motion;
2. static joints contribute to the *static* score, which computes their deviation from a predefined resting position.

Dynamic and static scores are combined to produce three levels:

1. low: both dynamic and static scores are low, producing the following feedback:
> "You are not moving very well!"

2. medium: either dynamic or static score is medium, producing the following feedback:
> "You are doing the exercise correctly, but you could do it better."

3. high: both dynamic and static scores are high, producing the following feedback:
>"You are moving very well!"

!!! failure
    - the feedback does not provide a real correction of the movement;
    - the *speed* of the movement is not directly taken into account;
    - the feedback can be positive even when a wrong movement is performed (for example, a different movement with the same moving joints as external rotation instead of internal and viceversa, or a random movement with dynamic joints moving within the range of motion and static joints not deviating enough from the resting position).

### Y1Q3

To overcome the drawbacks of Y1M5, in Y1Q3 the concept of dynamic and static joints is removed and the exercise is treated in its entirety as an action. Therefore the feedback is produced based on the following two layers:

1. [Action recognition](#action-recognition): which classifies the exercise and triggers the second layer if the predicted class equals the label of the exercise to perform. Otherwise the feedback produced is negative. This layer is fundamental as it guarantees a random or a wrong movement to not produce a positive feedback.
2. [Joint analysis](#joint-analysis): which compares the observed skeleton with a predefined template for that movement, differently based on the exercise. The comparison with a template skeleton allows us to analyze also the speed of the movement.

With this architecture, the feedback is extended and articulated in the following levels:

- positive feedback:
> "You are moving very well!"

- feedback for range of motion exercises:
    1. feedback on the speed:
      > "Move the arm faster/slower!"

    2. feedback on the range of motion:
      > "Move the arm further up/down!"

- feedback for reaching exercises:
> "You are not reaching the target!"

- negative feedback:
> "You are doing the wrong exercise. Please, repeat the movements I show you."

#### Action Recognition

The action recognition is carried out using a [Recurrent Neural Network (RNN) with Long Short-Term Memory (LSTM) cells](https://github.com/stuarteiffert/RNN-for-Human-Activity-Recognition-using-2D-Pose-Input).

- **Input to the network**:
*2D joints* of the upper body from the `skeletonRetriever`. The output of the `skeletonRetriever` is preferable to that of `yarpOpenPose`, as the `skeletonRetriever` unambiguously identifies a skeleton by tag, avoiding ambiguities due to the presence of several skeletons.

- **Preprocessing**:
The following operations are applied to the skeleton:

    1. normalization, such that the length of each segment is 1. This prevents having different results for different physiques;
    2. rototranslation, derived by the first observed skeleton;
    3. scale by a factor of 10, in order to have values between -1.0,1.0.

- **Training set**:
1 subject performs the following 6 exercises (i.e. 6 classes):

    1. `abduction-left`;
    2. `internal-rotation-left`;
    3. `external-rotation-left`;
    4. `reaching-left`;
    5. `static` (the subject remains steady);
    6. `random` (the subject moves randomly).

    The first 3 movements are repeated 10 times and the full exercise repeated 5 times.
    For the 4th movement, 4 targets to be reached are defined, distributed on the corners/center of a square, centered around the `shoulderLeft` of the subject. Each dataset was recorded from a frontal and a side view and can be found at this [link](https://github.com/robotology/assistive-rehab-storage/tree/master/dataset-action-2).
    Parameters used for training are the following:

    1. `n_hidden` = 22
    2. `n_steps` (temporal window) = 30
    3. `learning_rate` = 0.0005
    4. `batch_size` = 256
    5. `epochs` = 400

- **Validation set:**
The same subject, but previously unseen data, were used for testing the network.

- **Accuracy:**
We get an accuracy of 92.2% with the following confusion matrix:

<p align="center">
<img src=https://user-images.githubusercontent.com/9716288/49951737-ead66b80-fefa-11e8-831e-5fe666bf2bee.png width="650">
</p>

#### Joint analysis
This analysis is differentiated according to the exercises, which are classified as:

1. range of motion exercises (i.e. abduction, internal and external rotation);
2. reaching exercises.

##### Range of Motion exercises
The joints under analysis for these movements are: `elbowLeft`, `handLeft`.
These movements can produce two feedbacks, i.e. on the speed and on the range of motion. The feedback is provided according to a predefined hierarchy which prioritizes the speed, followed by the range of motion. Therefore, a positive feedback is produced only when both checks are fine.

1. **Speed: Fourier analysis**

We perform the Fourier transform of each component of the joints under analysis in a predefined temporal window, for both the observed and the template skeleton.
The difference in frequency is computed as `df = f_skeleton - f_template` and thus we can have two possible cases:

  1. `df > 0` => feedback: "Move the arm faster"
  2. `df < 0` => feedback: "Move the arm slower"

2. **Range of motion: Dynamic Time Warping (DTW) plus error statistical analysis**

The DTW is applied to each component of the joints under analysis, for both the observed and the template skeleton, allowing us to temporally align the signals to compare.
Once joints are aligned, the error between the observed and template joints under analysis is computed.         
A statistical analysis is carried out, which looks for tails in the error distribution. Tails can be identified using the skewness of the distribution.
Three cases can be identified:

  1. the skeleton and the template are moving similarly. Therefore the error in position is close to 0, generating a distribution centered around 0 (i.e. with low skewness):
  2. the skeleton has an higher range of motion than the template. Therefore the error will be positive, generating a distribution with a positive tail (i.e. with a positive skewness):
  3. the skeleton has a lower range of motion than the template. Therefore the error will be negative, generating a distribution with a negative tail (i.e. with a negative skewness):

##### Reaching
The joint under analysis for this movement is `handLeft`.
This movement produces a feedback related to how well a predefined target is reached.

1. **Statistical analysis of the reached target**

We define a sphere around the predefined target and consider the points of the template which fall within the sphere. Statistics of the distribution of the template points are extracted to describe how the points should be distributed around the target. We then compute the number of points of the observed skeleton which fall within the template distribution. If the number of inliers is above a predefined threshold, the target is considered reached and a positive feedback is produced, otherwise the target is considered not reached.

### Comparison

The following tables compare feedbacks produced by the two pipelines developed in Y1M5 and in Y1Q3 respectively, in response to the same movement.
Corrrect feedbacks are highlighted.

* `abduction-left`

|             | Y1M5                   | Y1Q3              |
| ------------| ---------------------- | ----------------- |
| **Correct** | **1. You are moving very well!**<br> **2. You are moving very well!** | **1. You are moving very well!**<br> **2. You are moving very well!**  |
| **Fast**    |  1.You are moving very well!<br> 2.You are moving very well! | **1. Move the left arm slower!** <br> **2. Move the left arm slower!** |
| **Slow**    | 1.You are moving very well!<br> 2.You are moving very well! | 1. Move the left arm further up! <br> 2. Move the left arm further up! |
| **Low ROM** |  1.You are moving very well!<br> 2.You are moving very well! | **1. Move the left arm further up!** <br> **2. Move the left arm further up!** |
| **Wrong**   |  1. You are doing the exercise correctly, but you could do it better. <br> 2. You are doing the exercise correctly, but you could do it better.  | **1. You are doing the wrong exercise.** <br> **2. You are doing the wrong exercise.** |

* `external-rotation-left`

|              | Y1M5                  | Y1Q3              |
| ------------ | --------------------- | ----------------------- |
| **Correct**  | 1. You are doing the exercise correctly, but you could do it better. <br> 2. You are doing the exercise correctly, but you could do it better. | 1. Move the left arm slower! <br> **2.You are moving very well!** |
| **Fast**     | 1.You are moving very well! <br> 2.You are moving very well!  | **1.Move the left arm slower!** <br> **2.Move the left arm slower!** |
| **Slow**     | **1. You are doing the exercise correctly, but you could do it better.** <br> 2. You are moving very well!  | 1. You are moving very well! <br> **2. Move the left arm faster!** |
| **Low ROM**  | 1.You are moving very well! <br> **2. You are doing the exercise correctly, but you could do it better.** | 1.You are moving very well! <br> 2.Move the left arm slower!  |
| **Wrong**    | 1. You are doing the exercise correctly, but you could do it better. <br> 2. You are doing the exercise correctly, but you could do it better. | **1. You are doing the wrong exercise.**<br> **2. You are doing the wrong exercise.** |

* `internal-rotation-left`

|              | Y1M5                   | Y1Q3              |
| -------------| ----------------------- | ----------------------- |
| **Correct**  | **1.You are moving very well!** <br> **2.You are moving very well!** | **1.You are moving very well!** <br> **2.You are moving very well!** |
| **Fast**     |  1.You are moving very well! <br> 2.You are moving very well! |  **1.Move the left arm slower!** <br> **2.Move the left arm slower!** |
| **Slow**     | 1.You are moving very well! <br> 2.You are moving very well! | 1.You are moving very well! <br> **2.Move the left arm faster!** |
| **Low ROM**  | **1.You are not moving very well!** <br> **2.You are not moving very well!** | **1. Move the left arm backwards!** <br> **2. Move the left arm backwards!** |
| **Wrong**    | **1.You are not moving very well!** <br> **2.You are not moving very well!** | **1. You are doing the wrong exercise.** <br>  **2. You are doing the wrong exercise.** |

!!! success
    - in Y1Q3 the correct feedback is provided for **_fast_** movements, while in Y1M5 the feedback is always positive, regardless the speed;
    - in Y1Q3 the correct feedback is provided for **_low ROM_** movements, while in Y1M5 the feedback is always positive (except for the external rotation, which generates a medium score since there are time instants where the range of motion is below the minimum one);
    - in Y1Q3 a negative feedback is provided for **_wrong_** movements, while in Y1M5 the feedback is generated by a medium score, (except for the internal rotation, due to the static score, which is low as static joints deviate a lot from the resting position).

!!! failure
    - in Y1Q3 a positive feedback is provided for **_slow_** movements. This occurs as the template skeleton moves slowly to emulate the movement shown by the robot and the feedback on the speed is triggered only by a very slow execution of the exercise;
    - in Y1Q3 a feedback on the speed is provided for an **_external rotation with low ROM_**, as the threshold on the frequency is set low to trigger also a feedback on slow movements.
