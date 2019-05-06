# Latest news

__May 6, 2019__ : Checkout our latest [release v0.3.0](https://github.com/robotology/assistive-rehab/releases/tag/v0.3.0)!

This is a major change which refactors the entire framework to deal also with feet, following up the use of [`BODY_25`](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/output.md) model of `OpenPose`.
The following is an example of skeleton with feet in 2D and 3D:

![screencast](https://user-images.githubusercontent.com/3738070/56869123-92f95680-69fc-11e9-8b54-3d463fc5c645.gif)

Changes include:

- `SkeletonStd` now includes `hip_center`, `foot_left`, `foot_right`:
    - `hip_center` is directly observed if available, otherwise is estimated as middle point between `hip_left` and `hip_right` (the same stands for `shoulder_center`);
    - `foot_left` and `foot_right` are detected as being the big-toe. If big-toe is not available, small-toe is used as fallback;
- `SkeletonWaist` has been removed in favor of the new `SkeletonStd`;
- optimization performed by `skeletonRetriever` is now extended also to lower limbs;
- modules previously relying on `SkeletonWaist` have been updated to use the new `SkeletonStd`;
- the new framework is compatible with the [Y1M5 demo](Y1M5.md), which was successfully tested online on the robot;
- the new framework is compatible with datasets recorded before the release, which can be reproduced by means of `yarpdataplayer`.

__May 6, 2019__ : Checkout our new [release v0.2.1](https://github.com/robotology/assistive-rehab/releases/tag/v0.2.1)!

What's new?

- the **action recognition** is now robust to **rotations**! The original network was trained with skeletons frontal to the camera, which is not guaranteed during a real interaction. The network has been re-trained with a wider training set, comprising synthetic rotations applied to real data around each axis, with variation of `10` degrees in a range of `[-20,20]` degrees. Also a variability on the speed was introduced in the training set, by considering the action performed at normal, double and half speed. We compared the accuracy of the previous and the new model for different rotations of the skeleton, and results show a high accuracy to a wider range for all axes:  

<p align="center"> ROTATION AROUND X </p>

| **Original** | **New** |
| ---------- | ----------------------------  |
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/52556551-31f2b700-2ded-11e9-9f65-02031283f2cd.png width="450"> </p> | <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/52556559-38812e80-2ded-11e9-889e-360e4f590ada.png width="450"> </p>  |

<p align="center"> ROTATION AROUND Y </p>

| **Original**  | **New** |
| ---------- | --------------------------  |
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/52556915-37043600-2dee-11e9-8474-2b436c803370.png width="450"> </p>  | <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/52556811-e68cd880-2ded-11e9-8e13-169faee42b55.png width="450"> </p> |

<p align="center"> ROTATION AROUND Z </p>

| **Original** | **New** |
| ---------- | ------------------------------|
| <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/52557916-286b4e00-2df1-11e9-9b37-e80c8b47a18c.png width="450"> </p> | <p align="center"> <img src=https://user-images.githubusercontent.com/9716288/52557930-3620d380-2df1-11e9-876b-04ee166701c5.png width="450"> </p> |

- the **skeleton** now also stores the **pixels** alongside the 3D points! This is very useful when using the skeleton for gaze tracking, as it avoids the transformation from the camera frame to the root frame of the robot required is using 3D information;
- the **offline report** is now **interactive**! The user can navigate through plots, zoom, pan, save:
![report2](https://user-images.githubusercontent.com/9716288/51993908-ffbd8d00-24af-11e9-8c61-86a2b58cfd7f.gif)

__January 28, 2019__ : Checkout our latest [tutorial](main_apps.md) on how to run the main applications on the robot R1!

__January 25, 2019__ : We are public now!

__December 21, 2018__ : Checkout the [latest release](https://github.com/robotology/assistive-rehab/releases/tag/v0.3.0) and the [the comparison with the first release](comparison_releases.md)!
