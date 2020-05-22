# TUG

This application has been developed during 2019 with the aim of expanding the handled scenarios to clinical test beyond rehabilitation.

The Timed Up and Go (TUG) measures the time the patient takes to stand up from a chair, walk for 3 meters and cross a line on the floor, turn, go back to the chair and sit down, as shown in the following picture:

![tug](https://user-images.githubusercontent.com/9716288/82028858-7034f100-9696-11ea-99e8-b23386b5c71b.gif)

!!! note
    This was realized within the simulation environment `gazebo`.

The application includes the following steps:

1. R1 engages the user, proposing a clinical test of the lower limbs, specifically the TUG;
2. R1 explains the test, navigating the environment to reach the lines on the floor and pointing at them;
3. R1 analyzes in real-time the correct execution of the test, verifying that the user crosses the finish line and goes back to the chair and simultaneously extracting real-time walking metrics (step length and width, walking speed and number of steps);
4. R1 eventually replies to questions during the test explanation and / or execution, if the user presses the provided button;
5. R1 produces a final report of the test, which includes the evaluated walking metrics.

!!! note
    The code implementing this application is tagged as [v0.5.0](https://github.com/robotology/assistive-rehab/releases/tag/v0.5.0).
