# Running Moveit in a demo simulation

In this section we'll look at how to use the moveit_interface in an rviz simulation.
Fore more details about moveit_interface, please read the [documentation](moveit_interface.md)

## Running the RViz similation

    cd ~/${CATKIN_WORKSPACE}
    . devel/setup.bash
    roslaunch yumi_moveit_config demo.launch

## Running the interface moveit_interface

    cd ~/${CATKIN_WORKSPACE}
    . devel/setup.bash
    roslaunch yumi_motion_api moveit_interface.launch
