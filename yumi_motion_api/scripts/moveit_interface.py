#!/usr/bin/env python3
# -*- coding: utf-8 -*-
""" A moveit interface to control Yumi."""

__author__ = "Maxence Grand"
__copyright__ = "TODO"
__credits__ = ["TODO"]
__license__ = "TODO"
__version__ = "0.0.1"
__maintainer__ = "Maxence Grand"
__email__ = "Maxence.Grand@univ-grenoble-alpes.fr"
__status__ = "Under Developpement"

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf

from math import pi, tau, dist, fabs, cos
import std_msgs.msg
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

IDLE_RIGHT = [
    1.6379728317260742,
    0.20191457867622375,
    -2.5927578258514404,
    0.538416862487793,
    2.7445449829101562,
    1.5043296813964844,
    1.7523150444030762
]
IDLE_LEFT = [
    -1.46564781665802,
    0.3302380442619324,
    2.507143497467041,
    0.7764986753463745,
    -2.852548837661743,
    1.659092664718628,
    1.378138542175293
]


def get_orientation_from_lego_orientation(horizontal=True):
    return [pi, 0, 0] if horizontal else [pi, 0, pi/2]

def init_arm(
        group_name,
        planner="ESTkConfigDefault",
        pose_reference_frame="yumi_body",
        replanning = False,
        position_tolerance=0.005,
        orientation_tolerance=0.005):
    """
    Init group arm

    @param group_name Name of the group arm
    @param planner Planner ID used
    @param pose_reference_frame The pose reference frame
    @param replaning True if the replanning is allowed
    @param position_tolerance Tolerance for the goal position
    @param orientation_tolerance  Tolerance for the goal orientation
    """
    group = moveit_commander.MoveGroupCommander(group_name)
    group.set_planner_id(planner)
    group.set_pose_reference_frame(pose_reference_frame)
    group.allow_replanning(replanning)
    group.set_goal_position_tolerance(position_tolerance)
    group.set_goal_orientation_tolerance(orientation_tolerance)
    return group

def create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad):
    """Creates a pose using euler angles

    Creates a pose for use with MoveIt! using XYZ coordinates and RPY
    orientation in radians

    @param x_p The X-coordinate for the pose
    @param y_p The Y-coordinate for the pose
    @param z_p The Z-coordinate for the pose
    @param roll_rad The roll angle for the pose
    @param pitch_rad The pitch angle for the pose
    @param yaw_rad The yaw angle for the pose
    @returns Pose
    @rtype PoseStamped
    """
    quaternion = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
    return create_pose(x_p, y_p, z_p, quaternion[0], quaternion[1], quaternion[2], quaternion[3])


def create_pose(x_p, y_p, z_p, x_o, y_o, z_o, w_o):
    """Creates a pose using quaternions

    Creates a pose for use with MoveIt! using XYZ coordinates and XYZW
    quaternion values

    @param x_p The X-coordinate for the pose
    @param y_p The Y-coordinate for the pose
    @param z_p The Z-coordinate for the pose
    @param x_o The X-value for the orientation
    @param y_o The Y-value for the orientation
    @param z_o The Z-value for the orientation
    @param w_o The W-value for the orientation
    @returns Pose
    @rtype PoseStamped
    """
    pose_target = geometry_msgs.msg.Pose()
    pose_target.position.x = x_p
    pose_target.position.y = y_p
    pose_target.position.z = z_p
    pose_target.orientation.x = x_o
    pose_target.orientation.y = y_o
    pose_target.orientation.z = z_o
    pose_target.orientation.w = w_o
    return pose_target

class Yumi:
    """
        Yumi
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = moveit_commander.RobotCommander()

        self.group_l = init_arm("left_arm")
        self.group_r = init_arm("right_arm")
        self.group_both = init_arm("both_arms")

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', 	moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        rospy.sleep(3)
        print("============ Yumi Initialized")


    ###################################################
    ############### Status Methods ####################
    ###################################################

    def get_current_pose(self,  right=True):
        """Gets the current pose

        Return the current pose of the selected arm

        @param right True if the selected arm is the right one
        @returns Pose
        @rtype PoseStamped
        """
        return self.group_r.get_current_pose() if right \
            else self.group_l.get_current_pose()

    def get_current_rpy(self,  right=True):
        """Gets the current oint values

        Return the current position of all joints of the selected arm as Euler angles

        @param right True if the selected arm is the right one
        @returns Orientation
        @rtype Orientation
        """
        return self.group_r.get_current_rpy() if right \
            else self.group_l.get_current_rpy()

    def get_current_joint_values(self,  right=True):
        """Gets the current joint values

        Return the current joint values of the selected arm

        @param right True if the selected arm is the right one
        @returns Joint values
        @rtype float[]
        """
        return self.group_r.get_current_joint_values() if right \
            else self.group_l.get_current_joint_values()

    def print_arm_status(self, right=True):
        """prints the status of the selected arm

        @param right True if the selected arm is the right one
        @returns None
        """
        sel = "right" if right else "left"
        print(f"Yumi's {sel} arm status")
        print(self.get_current_pose(right))
        print(self.get_current_rpy(right))
        print(self.get_current_joint_values(right))

    def print_status(self):
        """Prints yumi's status

        @return None
        """
        self.print_arm_status(right=True)
        self.print_arm_status(right=False)

    ###################################################
    ############## Movements Methods ##################
    ###################################################

    def go_to_pose_goal(self, target, right=True):
        """Plans and moves the selected arm to target

        Creates a plan to move a group to the given target.

        @param target: The pose to move to
        @param right True if the selected arm is the right one
        @returns None
        """
        euler = tf.transformations.euler_from_quaternion((target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w))

        if (right):
            arm = 'right'
            group = self.group_r
        else:
            arm = 'left'
            group = self.group_l

        rospy.loginfo('Planning and moving ' + arm + ' arm: Position: {' + str(target.position.x) + ';' + str(target.position.y) + ';' + str(target.position.z) +
                                            '}. Rotation: {' + str(euler[0]) + ';' + str(euler[1]) + ';' + str(euler[2]) + '}.')
        group.set_pose_target(target)
        success = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        group.clear_pose_targets()

    def go_to_joint_state(self, joint_goal, right=True):
        """Plans and moves the selected arm to the joint goal

        Creates a plan to move a group to the given joint goal.

        @param target: The joint goal
        @param right True if the selected arm is the right one
        @returns None
        """

        group = self.group_r if right else self.group_l
        group.go(joint_goal, wait=True)
        group.stop()

    def go_to_joint_state_both(self, joint_goal):
        """Plans and moves both arms to the joint goal

        Creates a plan to move a group to the given joint goal.

        @param right True if the selected arm is the right one
        @returns None
        """

        self.group_both.go(joint_goal, wait=True)
        self.group_both.stop()

    ###################################################
    ############### Grippers Methods ##################
    ###################################################

    def gripper_effort(self, effort, right=True):
        """Set gripper effort

        Sends an effort command to the selected gripper. Should be in the range of
        -20.0 (fully open) to 20.0 (fully closed)

        @param effort: The effort value for the gripper (-20.0 to 20.0)
        @param right True if the selected arm is the right one
        @returns None
        """
        gripper_id = 'right' if right else 'left'
        rospy.loginfo("Setting " + str(gripper_id) + " gripper to " + str(effort))
        rospy.loginfo('Setting gripper effort to ' + str(effort) + ' for '+ str(gripper_id) +' arm ')

        pubname = '/yumi/gripper_r_effort_cmd' if right else '/yumi/gripper_l_effort_cmd'
        pub = rospy.Publisher(pubname, std_msgs.msg.Float64, queue_size=10, latch=True)
        pub.publish(std_msgs.msg.Float64(effort))
        rospy.sleep(1)


class MoveItInterface(object):
    """MoveItInterface"""

    def __init__(self):
        super(MoveItInterface, self).__init__()

        self.scene = moveit_commander.PlanningSceneInterface()
        rospy.init_node("moveit_interface", anonymous=True)
        print("============ Node moveit_interface Initialized")
        rospy.sleep(2)

        self.yumi = Yumi()

        self.go_to_idle()

    def print_status(self):
        """Print Status"""
        self.yumi.print_status()

    ###################################################
    ############## Movements Methods ##################
    ###################################################

    def go_to(self, x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad, right=True):
        """Set end effector position

        Sends a command to MoveIt! to move to the selected position, in any way
        it sees fit.

        @param x_p: The X-coordinate for the pose
        @param y_p: The Y-coordinate for the pose
        @param z_p: The Z-coordinate for the pose
        @param roll_rad: The roll angle for the pose
        @param pitch_rad: The pitch angle for the pose
        @param yaw_rad: The yaw angle for the pose
        @param right True if the selected arm is the right one
        @returns None
        """
        target = create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
        self.yumi.go_to_pose_goal(target, right=right)
        self.print_status()

    def go_to_idle(self):
        """ Moves yumi to its idle position"""
        goal = []
        goal.extend(IDLE_LEFT)
        goal.extend(IDLE_RIGHT)
        self.yumi.go_to_joint_state_both(goal)
        self.print_status()

    ###################################################
    ############### Grippers Methods ##################
    ###################################################

    def close_gripper(self, right=True):
        """Close gripper

        Close the selected gripper.

        @param right True if the selected arm is the right one
        @returns None
        """
        self.yumi.gripper_effort(20,right=right)

    def open_gripper(self, right=True):
        """Open gripper

        Open the selected gripper.

        @param right True if the selected arm is the right one
        @returns None
        """
        self.yumi.gripper_effort(-20,right=right)

def main():
    try:
        interface = MoveItInterface()
        interactive_session(interface)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

def interactive_session(interface):
    print("Welcome to this interactive session")
    print("Follow the instructions to move Yumi.")
    try:
        while True:
            code = int(input("What instruction would you like to send Yumi? 0:Move/1:end "))
            if(code == 0):
                right = int(input("Which arm do you want to move? 0:right/1:left ")) == 0
                print("Let's start by giving the position:")
                x = float(input("x: "))
                y = float(input("y: "))
                z = float(input("z: "))
                h = int(input("Which lego orientation should Yumi take? 0:horizontal/1:vertical"))
                eulers = get_orientation_from_lego_orientation(horizontal = (h == 0))
                roll = eulers[0]
                pitch = eulers[1]
                yaw = eulers[2]
                interface.go_to(x, y, z, roll, pitch, yaw, right=right)
            else:
                print('End of the session')
                interface.go_to_idle()
                break
    except KeyboardInterrupt:
        print('End of the session')
        interface.go_to_idle()



if __name__ == "__main__":
    main()
