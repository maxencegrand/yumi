# Running Moveit in live

TODO

## Running the Robot

Please set the main pointer at the FlexPendant to main for the `Rob_L` and `Rob_R` tasks, then run them by presseng on the play button on the flexpendant lower right side for each of them (you can access them from `Program Editor`)

To run RVIZ with moveit:
- After you connect to YuMi (which should be running and motors on in Automatic mode):

    cd ~/ws_yumi && catkin b -DCMAKE_BUILD_TYPE=RELEASE && . devel/setup.bash
    cd src/yumi_ros/yumi_description/urdf/
    rosrun xacro xacro yumi.urdf.xacro arms_interface:=VelocityJointInterface grippers_interface:=EffortJoinInterface yumi_setup:=robot_centric -o yumi.urdf
    roslaunch yumi_moveit_config myLaunch.launch
