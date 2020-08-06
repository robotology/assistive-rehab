# How to use the gazebo plugin

This tutorial will show you how to use the `gazebo` plugin we developed for an animated model, which is called `actor` in `gazebo`.

As shown in this [gazebo tutorial](http://gazebosim.org/tutorials?tut=actor&cat=build_robot), an `actor` in `gazebo` is a model which can be animated using [`.dae`](http://gazebosim.org/tutorials?tut=actor&cat=build_robot#Skeleton) files and [scripted trajectories](http://gazebosim.org/tutorials?tut=actor&cat=build_robot#Scriptedtrajectories).  
Our plugin extends the `actor` class, allowing the user to play and stop single animations or the whole script, to change the speed on the fly, to reach targets in the environment.

## Dependencies

After [installing](https://robotology.github.io/assistive-rehab/doc/mkdocs/site/install/) `assistive-rehab`, you will need the following dependencies:

- [gazebo](https://github.com/vvasco/gazebo): for running the virtual environment;
- [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins): for exposing YARP interfaces in `gazebo`;

## Preparing your environment

The first step you need to take is to prepare your environment.

Installing `assistive-rehab` will result in the shared library `libgazebo_assistiverehab_tuginterface.so`, which can be included in a `gazebo` simulation.

Following [this](http://gazebosim.org/tutorials?tut=components&cat=get_started) gazebo tutorial, you need to let `gazebo` know where the plugin shared library is located. For doing this, you will need to set the following environment variables:

- `GAZEBO_PLUGIN_PATH`: has to point to the folder where you installed the shared library;
- `GAZEBO_MODEL_PATH`: has to point to the folder including the actor `.dae` files;

!!! info "gazebo models"
    `gazebo` currently provides several `.dae` animations, which can be found [here](https://github.com/osrf/gazebo/tree/gazebo11/media/models).

- `GAZEBO_RESOURCE_PATH`: has to point to the folder including your world application.

## How to include the plugin in your world

To include the plugin in your world, you can add the highlighted lines to the actor tag:

``` xml hl_lines="26-28"
<actor name="actor">
  <skin>
    <filename>stand.dae</filename>
  </skin>
  <animation name="stand">
    <filename>stand.dae</filename>
  </animation>
  <animation name="sit_down">
    <filename>sit_down.dae</filename>
  </animation>
  <animation name="stand_up">
    <filename>stand_up.dae</filename>
  </animation>
  <animation name="walk">
    <filename>walk.dae</filename>
    <interpolate_x>true</interpolate_x>
  </animation>
  <script>
    <loop>false</loop>
    <auto_start>false</auto_start>
    <trajectory id="0" type="stand"/>
    <trajectory id="1" type="sit_down"/>
    <trajectory id="2" type="stand_up"/>
    <trajectory id="3" type="walk"/>
  </script>
  <plugin name='tug_interface' filename='libgazebo_assistiverehab_tuginterface.so'>
    <yarpConfigurationFile>model://tugInterface.ini</yarpConfigurationFile>
  </plugin>
</actor>
```

This example snippet generates an actor associated to three animations, `stand.dae`, `sit_down.dae`, `stand_up.dae` and `walk.dae`.
The plugin loads the [`tugInterface.ini`](https://github.com/robotology/assistive-rehab/blob/feat/doc-v0.5.0/app/gazebo/tug/tugInterface.ini) configuration file and opens a rpc port named by default `/tug_input_port`, which gives you access to a set of thrift services, described in the next section.

!!! important
    To use the plugin you will need to have an actor with at least one animation associated.

!!! note
    We are going to refer to this snippet in next section to see the plugin functionalities, but you can personalize the actor according to your needs.

## Using the plugin

To start using the plugin, first open a terminal and run `yarpserver`.
Open a new terminal and type `gazebo` followed by the name of your scenario.

!!! important "Remember"
    `gazebo` looks for world files in the `$GAZEBO_RESOURCE_PATH` environment variable, as described at the [beginning](#preparing-your-environment) of this tutorial.

Now you should see an actor standing on your screen:

![default_gzclient_camera(1)-2020-05-18T15_36_11 353681](https://user-images.githubusercontent.com/9716288/82219308-6244d180-991d-11ea-9caa-ab07f382b74e.jpg)

You can start playing with the plugin!

Open a terminal and type `yarp rpc /tug_input_port`.

### Playing the animations

Type `play` on the terminal. All the animations are going to be played in a row, following the sequence defined by the ids in the trajectories as defined [here](#how-to-include-the-plugin-in-your-world):

![actor](https://user-images.githubusercontent.com/9716288/82221362-337c2a80-9920-11ea-9d24-7999e39af9ff.gif)

Additionally, you can play the single animations associated to the actor or play the whole script from the specified animation:

|  <p align="center"> **PLAY SINGLE ANIMATION**  </p>  | <p align="center"> **PLAY FROM ANIMATION**  </p>  |
| ---------------------------------------------------- | ------------------------------------------------- |
| `play stand` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82222196-3d525d80-9921-11ea-8e8f-5248243062ed.gif"> </p> | `play stand -1 true` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82225129-1007ae80-9925-11ea-9d9d-0f55a0e4a5c9.gif"> </p> |
| `play sit_down` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82223329-cddd6d80-9922-11ea-8ada-f6b7d7b19e70.gif"> </p> | `play sit_down -1 true` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82225178-23b31500-9925-11ea-8165-ce3b8ec0bcfe.gif"> </p> | <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82225178-23b31500-9925-11ea-8165-ce3b8ec0bcfe.gif"> </p> |
| `play stand_up` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82223561-1a28ad80-9923-11ea-8632-9ea133b16570.gif"> </p> | `play stand_up -1 true` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82225209-3299c780-9925-11ea-968c-cfd4d427b593.gif"> </p> |
| `play walk` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82223615-2a408d00-9923-11ea-92af-dc77471237ed.gif"> </p> | `play walk  -1 true` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82223615-2a408d00-9923-11ea-92af-dc77471237ed.gif"> </p> |

!!! note "Retrieving animations"
    You can retrieve the list of animations associated to your actor using the rpc command `getAnimationList`.

!!! example
    Playing a single animation of the script is useful when you need a condition to be accomplished before the specific animation is played.
    For example, in the TUG scenario, we want the actor to sit or to start walking only when the robot gives the related command.
    You can check it out [here](tug_demo.md).

### Changing walking parameters

The path followed during the `walk` animation is specified by `targets` in the [`tugInterface.ini`](https://github.com/robotology/assistive-rehab/blob/feat/doc-v0.5.0/app/gazebo/tug/tugInterface.ini) configuration file: this is a matrix containing the poses to reach in the form `x y theta`.

!!! important "Reference frame"
    The targets are defined with respect to the gazebo world frame, thus `X` pointing forward (with respect to the human model), `Y` pointing left, `Z` pointing up.

For [our application](TUG.md), we need the human model to reach the target and go back to the initial position. You can control the trajectory through the following rpc commands:

|  <p align="center"> **DESCRIPTION**  </p>  | <p align="center"> **OUTPUT**  </p>  |
| ------------------------------------------ | ------------------------------------ |
| <p align="center">  **`setTarget x y theta`**: <br> providing `x y theta`, when playing the `walk` animation, the human model will reach the new specified target and go back. </p> | - `setTarget 3.5 2.0 10.0` <br> - `play walk` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82234677-0173c400-9932-11ea-8836-a2767157430f.gif"> </p> |
| <p align="center">  **`goToSeq (x1 y1 theta1 ... xN yN thetaN)`**: <br> providing the list of `xi yi thetai`, the human model reaches the specified waypoints. </p> | - `goToSeq (3.0 2.0 20.0 5.0 0.0 -20.0)` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82229490-cc17a800-992a-11ea-867f-efc40e2e13e6.gif"> </p> |

!!! note
    The `goToSeq` is a blocking service: only when the last target is reached, an ack is returned.

!!! tip
    The interface provides two additional services, `goTo` and `goToWait` which are the punctual versions of `goToSeq`, respectively non blocking and blocking.

Finally, you can set the speed of the walking animation through the command `setSpeed`:

|  <p align="center"> **WALKING SLOWER**  </p>  | <p align="center"> **WALKING FASTER**  </p>  |
| ---------------------------------------------------- | ------------------------------------------------- |
| - `setSpeed 0.5` <br> - `play walk` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82234799-249e7380-9932-11ea-9978-81601d8ad1ac.gif"> </p> | - `setSpeed 2.0` <br> - `play walk` <p align="center"> <img src="https://user-images.githubusercontent.com/9716288/82234837-3253f900-9932-11ea-9311-ba319070bdea.gif"> </p> |

### Additional services

The additional services provided are the following:

- `playFromLast`: to play the animation from the last stop. A stop can be provided while an animation is being played. With this command, you will be able to start the script exactly from where you stopped.

!!! note
     The whole script is played by default. The command `playFromLast false` plays only the animation coming after last stop.

- `getState`: to know the current animation being played;
- `pause`: to pause the actor for a specified time or for an unlimited time (if not specified and until the `play` command is provided).
