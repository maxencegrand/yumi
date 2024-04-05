#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

""" A moveit interface to control Yumi."""

__author__ = "Belal Hmedan"
__copyright__ = "TODO"
__credits__ = ["TODO"]
__license__ = "TODO"
__version__ = "0.0.1"
__maintainer__ = "Maxence Grand"
__email__ = "Maxence.Grand@univ-grenoble-alpes.fr"
__status__ = "Under Developpement"

import sys
import copy
import rospy

from math import pi, tau, dist, fabs, cos, degrees
import std_msgs.msg
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
from moveit_commander.conversions import pose_to_list
import moveit_commander

def init_arm(
        group_name,
        planner="RRTConnect",
        pose_reference_frame="yumi_body",
        replanning = True,
        position_tolerance=0.0005,
        orientation_tolerance=0.0005,
        planning_attempts=100,
        planning_time=20,
        max_velocity=1,
        max_acceleration=1):
    """
    Init group armdfg

    @param group_name Name of the group arm
    @param planner Planner ID used
    @param pose_reference_frame The pose reference frame
    @param replaning True if the replanning is allowed
    @param position_tolerance Tolerance for the goal position
    @param orientation_tolerance  Tolerance for the goal orientation
    @param planning_attempts Number of planning planning_attempts
    @param planning_time Planning Ti;e
    @param max_velocity Velovity max 0<= <=1
    @param max_acceleration Acceleration max 0<= <=1
    """
    group = moveit_commander.MoveGroupCommander(group_name)
    print(group.get_joints())
    group.set_planner_id(planner)
    group.set_pose_reference_frame(pose_reference_frame)
    group.allow_replanning(replanning)
    group.set_goal_position_tolerance(position_tolerance)
    group.set_goal_orientation_tolerance(orientation_tolerance)
    group.set_num_planning_attempts(planning_attempts)
    group.set_planning_time(planning_time)
    group.set_max_velocity_scaling_factor(max_velocity)
    group.set_max_acceleration_scaling_factor(max_acceleration)
    return group

class Yumi():
    """Yumi Class (Simulated)"""

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = RobotCommander('robot_description')
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

    def execute_plan(self, plan, right=True):
        """Plans and moves the selected arm to target

        Creates a plan to move a group to the given target.

        @param target: The pose to move to
        @param right True if the selected arm is the right one
        @returns None
        """

        if (right):
            arm = 'right'
            group = self.group_r
        else:
            arm = 'left'
            group = self.group_l

        group.execute(plan, wait=True)
        group.stop()
        rospy.sleep(3)

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

        rospy.loginfo('Planning and moving '
                + arm + ' arm: Position: {'
                + str(target.position.x) + ';' + str(target.position.y) + ';'
                 + str(target.position.z) +
                '}. Rotation: {' + str(euler[0]) + ';' + str(euler[1]) + ';' + str(euler[2]) + '}.')

        group.set_pose_target(target)
        group.set_goal_position_tolerance(0.001)
        group.go(wait=True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        group.clear_pose_targets()
        rospy.sleep(3)

    def go_to_joint_state(self, target, right=True):
        """Plans and moves the selected group to the joint goal

        Creates a plan to move a group to the given joint goal.

        @param target: The joint goal
        @param right True if the selected arm is the right one
        @returns None
        """

        if (right):
            arm = 'right'
            group = self.group_r
        else:
            arm = 'left'
            group = self.group_l

        group.go(target,wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()
        rospy.sleep(3)

    def move_linear(self, z_value, right=True):
        """Up the selected arm"""
        group = self.group_r if (right) else self.group_l
        wpose = copy.deepcopy(group.get_current_pose()).pose
        print(wpose.position.z)
        wpose.position.z = z_value
        print(wpose.position.z)
        waypoints = []
        waypoints.append(wpose)
        (plan, fraction) = group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        self.execute_plan(plan, right=right)

    def up(self, z_value, right=True):
        """Up the selected arm"""
        group = self.group_r if (right) else self.group_l
        wpose = copy.deepcopy(group.get_current_pose()).pose
        print(wpose.position.z)
        wpose.position.z = z_value
        print(wpose.position.z)
        waypoints = []
        waypoints.append(wpose)
        (plan, fraction) = group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        self.execute_plan(plan, right=right)

    def down(self, z_value, right=True):
        """Down the selected arm"""
        group = self.group_r if (right) else self.group_l
        wpose = copy.deepcopy(group.get_current_pose()).pose
        # print(wpose.position.z)
        wpose.position.z = z_value
        # print(wpose.position.z)
        waypoints = []
        waypoints.append(wpose)
        (plan, fraction) = group.compute_cartesian_path(
            waypoints, 0.01, 5.0  # waypoints to follow  # eef_step
        )  # jump_threshold
        print(fraction)
        self.execute_plan(plan, right=right)

    def rotate(self, x_o, y_o, z_o, w_o, right=True):
        """Rotates selected arm"""
        group = self.group_r if right else self.group_l
        wpose = copy.deepcopy(group.get_current_pose()).pose
        wpose.orientation.x = x_o
        wpose.orientation.y = y_o
        wpose.orientation.z = z_o
        wpose.orientation.w = w_o
        self.go_to_pose_goal(wpose, right=right)

    ###################################################
    ############## Movements Methods ##################
    ###################################################
    def calibrateGrippers(self):
        """calibrate grippers"""
        return

    def effort(self, effort_value, right=True):
        """gripper efforts"""
        return

HOME = {
    "left": [0.0, -2.2699, 2.3561, 0.5224, 0.0033, 0.6971, 0.0035],
    "right": [0.0042, -2.2704, -2.353, 0.5187, -0.0048, 0.6965, 0.0051]
    # "right":[1.1015723699824334, -1.7478509832404054, -1.481914219561203, 0.18485387391245572, 0.7643431899827013, -0.8809056632052379, 0.09463194696309844]
}

def get_orientation_from_lego_orientation(horizontal=True):
    return [pi, 0, 0] if horizontal else [pi, 0, pi/2]

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
    # pose_target.orientation.x = -0.11214601049205945
    # pose_target.orientation.y = 0.8430209354939565
    # pose_target.orientation.z = 0.06825336211029923
    # pose_target.orientation.w = 0.5216133177079065

    return pose_target

PATCH_Z_VALUE = -.1

class MoveItInterface(object):
    """MoveItInterface"""

    def __init__(self):
        super(MoveItInterface, self).__init__()
        # self.sys_init()
        self.motion_init()


    def print_status(self):
        """Print Status"""
        self.yumi.print_status()

    # def sys_init(self):
    #     """
    #     Initialize the system, auto_mode, motors_on, rapid_running,
    #     egm_settings_adjusted
    #     """
    #     auto_mode = True # system_info.auto_mode #
    #     # rapid_running = system_info.rapid_running
    #     motors_on = False # system_info.motors_on #
    #     # print(auto_mode, motors_on)
    #
    #     if(not auto_mode):
    #         print("xxxxxxxxxxxxx Robot in Manual Mode! Can't Initialize! xxxxxxxxxxxxx")
    #         sys.exit()
    #     else:
    #         # Reset Program Pointer to main and Start RAPID
    #         rospy.wait_for_service('/yumi/rws/pp_to_main')
    #         rospy.wait_for_service('/yumi/rws/stop_rapid')
    #         rospy.wait_for_service('/yumi/rws/start_rapid')
    #         # set_motors_off= rospy.ServiceProxy("/yumi/rws/set_motors_off",TriggerWithResultCode)
    #         self.stop_rapid    = rospy.ServiceProxy("/yumi/rws/stop_rapid"   , TriggerWithResultCode)
    #         self.pp_to_main    = rospy.ServiceProxy("/yumi/rws/pp_to_main"   , TriggerWithResultCode)
    #         self.start_rapid   = rospy.ServiceProxy("/yumi/rws/start_rapid"  , TriggerWithResultCode)
    #         # Call the functionalities
    #         self.stop_rapid()
    #         self.pp_to_main()
    #         time.sleep(1)
    #         self.start_rapid()
    #         # Turn Motors On
    #         if(not motors_on):
    #             rospy.wait_for_service('/yumi/rws/set_motors_on')
    #             self.set_motors_on = rospy.ServiceProxy("/yumi/rws/set_motors_on", TriggerWithResultCode)
    #             self.set_motors_on()

    def motion_init(self):
        """Init motion interface"""
        self.scene = PlanningSceneInterface()
        rospy.init_node("moveit_interface", anonymous=True)
        print("============ Node moveit_interface Initialized")
        rospy.sleep(2)

        self.yumi = Yumi()
        self.add_scene_table()
        # self.add_block(
        #     Block(0,[4,4],[5,4],[5,5],[4,5])
        # )
        # self.add_block(
        #     Block(1,[24,11],[25,11],[25,12],[24,12])
        # )
        # self.add_block(
        #     Block(2,[45,23],[48,23],[48,24],[45,24])
        # )
        # rospy.sleep(3)
        self.calibrateGrippers()


    # def add_block(self, block):
    #     """Add block in the scene"""
    #     block_name = f"block_{block.id}"
    #     self.scene.remove_world_object(block_name)
    #     self.scene.add_box(
    #         block_name,
    #         block.get_real_position(self.yumi),
    #         block.get_dimension())

    def add_scene_table(self):
        """Add a table on the scene"""
        self.scene.remove_world_object("table")
        p = PoseStamped()
        p.header.frame_id = self.yumi.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = 0.405
        p.pose.position.y = 0.000
        p.pose.position.z = 0.050 # Table Height
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        self.scene.add_box("table", p, (0.510, 1.000, 0.001))

    ###################################################
    ############## Movements Methods ##################
    ###################################################

    def go_to(self, x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad, right=True, status=False):
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
        z_p += PATCH_Z_VALUE
        target = create_pose_euler(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad)
        self.yumi.go_to_pose_goal(target, right=right)
        if(status):
            self.print_status()

    def go_to_home(self):
        """ Moves yumi to its calib position"""
        self.yumi.go_to_joint_state(HOME["left"], right=False)
        self.yumi.go_to_joint_state(HOME["right"], right=True)
        self.print_status()

    def rotate(self, roll_rad, pitch_rad, yaw_rad, right=True):
        """rotates the selected arm"""
        x_o, y_o, z_o, w_o = tf.transformations.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        self.yumi.rotate(x_o, y_o, z_o, w_o, right=right)

    def down(self, z_value, right=True):
        """Down arm"""
        self.yumi.move_linear(z_value + PATCH_Z_VALUE, right=right)

    def up(self, z_value, right=True):
        """Down arm"""
        self.yumi.move_linear(z_value + PATCH_Z_VALUE, right=right)

    def move_linear(self, z_value, right=True):
        """Down arm"""
        self.yumi.move_linear(z_value + PATCH_Z_VALUE, right=right)

    ###################################################
    ############### Grippers Methods ##################
    ###################################################

    def calibrateGrippers(self):
        """Calibrates Yumi's grippers"""
        self.yumi.calibrateGrippers()

    def close_gripper(self, right=True):
        """Close gripper

        Close the selected gripper.

        @param right True if the selected arm is the right one
        @returns None
        """
        self.yumi.effort(0,right=right)

    def open_gripper(self, right=True):
        """Open gripper

        Open the selected gripper.

        @param right True if the selected arm is the right one
        @returns None
        """
        self.yumi.effort(20,right=right)

    def grasp(self, right=True):
        """close gripper to grasp

        Open the selected gripper.

        @param right True if the selected arm is the right one
        @returns None
        """
        self.yumi.effort(15,right=right)

    ###################################################
    ############### Services Methods ##################
    ###################################################

    def pick(self, block, right=True):
        """Pick the block"""

        rospy.loginfo(f'Pick Block {block.id}: Step 1: Open the gripper ')
        self.open_gripper(right=right)

        # rospy.loginfo(f'Pick Block {block.id}: Step 2: Rotate the gripper ')
        # self.rotate(roll_rad, pitch_rad, yaw_rad)

        rospy.loginfo(f'Pick Block {block.id}: Step 2: Move the gripper above the block')
        roll_rad, pitch_rad, yaw_rad = get_orientation_from_lego_orientation(block.is_horizontal)
        pos_block = copy.deepcopy(block.get_real_position(self.yumi)).pose
        x_p = pos_block.position.x
        y_p = pos_block.position.y
        z_p = pos_block.position.z + .30 #The gripper  remains 30cm above the block
        self.go_to(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad, right=right)

        rospy.loginfo(f'Pick Block {block.id}: Step 3: Down to the block')
        # z_value = z_p - pos_block.position.z + PATCH_Z_VALUE
        self.move_linear(pos_block.position.z, right=right)

        rospy.loginfo(f'Pick Block {block.id}: Step 4: Grasp the block')
        self.grasp(right=right)

        rospy.loginfo(f'Pick Block {block.id}: Step 5: Up the Gripper')
        self.move_linear(z_p, right=right)

    def place(self, block, right=True):
        """Place the block"""

        rospy.loginfo(f'Place Block {block.id}: Step 1: Move the gripper above the new block position')
        roll_rad, pitch_rad, yaw_rad = get_orientation_from_lego_orientation(block.is_horizontal)
        pos_block = block.get_real_position(self.yumi).pose
        x_p = pos_block.position.x
        y_p = pos_block.position.y
        z_p = pos_block.position.z + .30 #The gripper  remains 30cm above the block
        z_p_push = pos_block.position.z + .20 #The gripper  remains 3cm above the block tu push it
        self.go_to(x_p, y_p, z_p, roll_rad, pitch_rad, yaw_rad, right=right)

        rospy.loginfo(f'Place Block {block.id}: Step 2: Down to the new block position')
        z_value = z_p - pos_block.position.z
        self.move_linear(pos_block.position.z, right=right)

        rospy.loginfo(f'Place Block {block.id}: Step 3: Open the gripper ')
        self.open_gripper(right=right)

        rospy.loginfo(f'Place Block {block.id}: Step 4: Up the Gripper')
        self.move_linear(z_p_push, right=right)

        rospy.loginfo(f'Place Block {block.id}: Step 5: Push the block')
        self.move_linear(pos_block.position.z, right=right)

        rospy.loginfo(f'Place Block {block.id}: Step 6: Up the Gripper')
        self.move_linear(z_p, right=right)

        rospy.loginfo(f'Place Block {block.id}: Step 7: Go to home')
        self.go_to_home()

    def pick_n_place(self, block_current, block_next):
        """Pick and Place a block"""
        #Choose the arm
        right_arm = block_current.top_left[0] <= 24
        arm = 'right' if right_arm else 'left'
        rospy.loginfo(f'Pick and Place Block {block_current.id} performed using {arm} arm')

        self.pick(block_current, right = right_arm)
        self.place(block_next, right = right_arm)


class Block:
    """
        Block
    """

    def __init__(self, id, top_left, top_right, bottom_left, bottom_right, level=0):
        """
        Constructs a Block

        @param id: Block id
        @top_left: top left corner coordinates
        @top_right: top right corner coordinates
        @bottom_left: bottom left corner coordinates
        @bottom_right: bottom right corner coordinates
        @level: Block level
        """
        self.id = id
        self.top_left = top_left
        self.top_right = top_right
        self.bottom_left = bottom_left
        self.bottom_right = bottom_right
        self.level = level

    def is_horizontal(self):
        """
        Check if the block is horizontal
        Square block are consider as horizon

        @return True if the the block is horizontal
        """
        return self.top_right[0] - self.top_left[0] >= self.bottom_right[1] - self.top_right[1]

    def center(self):
        """Block center"""
        return (
            (self.top_right[0] + self.top_left[0])/2,
            (self.bottom_right[1] + self.top_right[1])/2
        )

    def get_mm(x):
        """Convert abstract x into mm position on the lego plate"""
        return x * 16

    def get_real_position(self, yumi):
        """
        Get the center position in the Yumi coordinates system
        """
        center = self.center()

        p = PoseStamped()
        p.header.frame_id = yumi.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = 0.280 + Block.get_mm(center[1])/1000
        p.pose.position.y = -0.380 + Block.get_mm(center[0])/1000
        p.pose.position.z = 0.008*(self.level+1) + 0.055
        p.pose.orientation.x = 0.0
        p.pose.orientation.y = 0.0
        p.pose.orientation.z = 0.0
        p.pose.orientation.w = 1.0
        return p

    def get_dimension(self):
        """block dimension"""
        return [
            Block.get_mm(self.bottom_right[1] - self.top_right[1])/1000,
            Block.get_mm(self.top_right[0] - self.top_left[0])/1000,
            0.016*(1+self.level)
        ]

def main():
    try:
        interface = MoveItInterface()
        blue = Block(
            0,
            [4,4],
            [5,4],
            [5,5],
            [5,5]
        )

        blue_next = Block(
            0,
            [24,12],
            [25,12],
            [25,13],
            [24,13]
        )
        interface.pick_n_place(blue, blue_next)
        yellow = Block(
            1,
            [38,11],
            [39,11],
            [39,12],
            [38,12]
            )
        yellow_next = Block(
            1,
            [28,11],
            [29,11],
            [29,12],
            [28,12]
            )
        interface.pick_n_place(yellow, yellow_next)
        # interactive_session(interface)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

def interactive_session(interface):
    print("Welcome to this interactive session")
    print("Follow the instructions to move Yumi.")
    try:
        while True:
            try:
                code = int(input("What instruction would you like to send Yumi? 0:Move/1:Gripper/2:end "))
                if(code == 0):
                    right = int(input("Which arm do you want to move? 0:right/1:left ")) == 0
                    print("Let's start by giving the position:")
                    x = float(input("x: "))
                    y = float(input("y: "))
                    z = float(input("z: "))
                    h = int(input("Which lego orientation should Yumi take? 0:horizontal/1:vertical"))
                    eulers = get_orientation_from_lego_orientation(horizontal = (h == 0))
                    # eulers = [0,0,0]
                    roll = eulers[0]
                    pitch = eulers[1]
                    yaw = eulers[2]
                    interface.go_to(x, y, z, roll, pitch, yaw, right=right)
                elif(code == 1):
                    right = int(input("Which gripper do you want to use? 0:right/1:left ")) == 0.
                    if( int(input('0:Open/1:close')) == 0):
                        interface.open_gripper(right=right)
                    else:
                        interface.close_gripper(right=right)
                else:
                    print('End of the session')
                    interface.go_to_home()
                    break
            except Exception:
                print(e)
                print("ERROR")
                print('End of the session')
                interface.go_to_home()
                break
    except KeyboardInterrupt:
        print('End of the session')
        interface.go_to_home()

if __name__ == "__main__":
    main()
