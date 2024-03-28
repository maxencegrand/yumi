# Controlling Yumi using moveit_interface

Before you start, please read the moveit package [wiki](https://ros-planning.github.io/moveit_tutorials/).

Yumi is controlled using the moveit package. This package provides motion planners for moving Yumi.There are several ways to use this package.

1. __Join States:__ The radian target position of each joint is given to moveit, which then computes a plan to achieve the target.
2. __End-Effector Position and Orientation:__ The position and orientation of the End-Effector, i.e. the gripper of one of the two arms, can be given as a target pose. The position is a tuple $`(x,y,z)`$ in [cartesian space](https://en.wikipedia.org/wiki/Cartesian_coordinate_system), and the orientation is a tuple of [quaternions](https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation) $`(w,x,y,z)`$. This target pose is used by moveit to compute the target joint state and plan the movement of the Yumi arm.
3. __Cartesian Path:__ A set of waypoints for the End-Effector are provided to moveit, which generates a planning passing through these points. We do not use this technique.

## Controlling Arm Movements

Three controllers are available to move the moveit arms:
1. Only the right arm. We use the End-Effector Position and Orientation moving strategies is used to move the right arm.
2. Only the left arm. We use the End-Effector Position and Orientation moving strategies is used to move the left arm.
3. Both arms. For simplicity's sake, we'll use this controller only when we want that Yumi reaches its idle position using joint states moving strategies.

### Computing the End-Effector Position using Lego Position

TODO

### Computing the End-Effector Orientation using Lego Position

To pick up a lego, whatever its size, the orientation of the pliers must always be the opposite of the lego's orientation. This way, whatever the size of the lego, Yumi will pick it up by its thinnest side.

To compute the orientation of the End-Effector, we start by calculating the corresponding [euler angle](https://en.wikipedia.org/wiki/Euler_angles) (Roll, Pitch, Yaw):
* $`(\pi,0,0)`$ if the lego is horizontal
* $`(\pi,0,\pi/2)`$ if the lego is vertical

This euler angle is then converted into quaternions and used to compute the target pose.

## Controlling Grippers
TODO
