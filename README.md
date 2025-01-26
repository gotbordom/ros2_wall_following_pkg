# Wall Follower

## Objective

To write a wall following algorithm in C++ using the ROS2 Framework

## Design Choices

### Main Loop

The main loop of this code, like many ROS projects utilizes the ROS framework to simplifly the main flow of code. The pattern here will look ideally like the following.

<p align="center">
  <picture>
    <img src="https://github.com/gotbordom/ros2_basics/blob/master/src/wall_following_pkg/resources/Wall%20Follower%20Main.png?raw=true" alt="Wall Follower Main" style="width:25%; max-width:600px; height:auto;">
  </picture>
</p>

### Callback Functions

For each of the: Laser scan sub, Odom sub, Geometry pub, Logger pub, and Controller timer there will be a callback function. The code flow for each are shown here.

#### Laser Scan Callback

The main job of this callback is to obtain the closest points to the walls being followed. The laser scanner callback is broken into the initial setup and the main loop.

##### Setup

In this stage we store important variables:

- Minimum / Maximum valid range of data
- Minimum / Maximum valid angles
- Total range size
- indexes for each sub range we need to pay attention to
  - Front of robot
  - Left side of robot
  - Right side of robot

Once all of the initial data is setup we no longer need to run though this stage

##### Main Loop

This is were we find:

- The minimum values from each range
- Store and/or update any values

Since this is a callback method it will be called everytime a new laser scan message is published to the correct topic being subscribed to.

<p align="center">
  <picture>
    <img src="https://github.com/gotbordom/ros2_basics/blob/master/src/wall_following_pkg/resources/Wall%20Follower%20Laser%20Scan%20Callback.png?raw=true" alt="Wall Follower Main" style="width:25%; max-width:600px; height:auto;">
  </picture>
</p>

#### Odom Callback

The main job of this callback is to create an estimate of how close / far we are from the wall being followed, and to udpate the state. The odom callback is also broken into setup and the main loop.

##### Setup

Get and calculate any required variables such as current x, y, yaw and previous x, y, yaw values.

This again should only ever be called once.

##### Main loop

The main loop listens to the Odom publisher, updating the current and previous positions and calculating and storing the estimated distances from the minimum range values that were determined from the laser scanner callback.

<p align="center">
  <picture>
    <img src="https://github.com/gotbordom/ros2_basics/blob/master/src/wall_following_pkg/resources/Wall%20Follower%20Odom%20Callback.png?raw=true" alt="Wall Follower Main" style="width:25%; max-width:600px; height:auto;">
  </picture>
</p>

#### Controller Callback

This controller is a fair bit more complicated and broken into a few graphs. The aim of callback is to be attached to the timer to called periodically to publish the next angular and linear velocities to publish to the robot motors.

This controller handles:

- If a user didn't select one, picking the wall to follow
- Using the data produced from Odom to determine if it needs to drive, turn or stop.

<p align="center">
  <picture>
    <img src="https://github.com/gotbordom/ros2_basics/blob/master/src/wall_following_pkg/resources/Wall%20Follower%20Control%20Callback.png?raw=true" alt="Wall Follower Main" style="width:25%; max-width:600px; height:auto;">
  </picture>
</p>

## Acknowledgments

This work was created by a tutorial in https://app.theconstruct.ai/ very similar diagrams were found in one of their videos.
