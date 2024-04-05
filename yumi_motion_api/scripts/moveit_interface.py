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

from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf
import time

from math import pi, tau, dist, fabs, cos, degrees
import std_msgs.msg
from geometry_msgs.msg import *
from sensor_msgs.msg import *
from moveit_msgs.msg import *
from moveit_commander import *
from moveit_commander.conversions import pose_to_list
from abb_rapid_sm_addin_msgs.srv import *
from abb_robot_msgs.srv import *
from abb_robot_msgs.msg import *
from controller_manager_msgs.srv import *

IDLE = {
    "right": [
        1.6379728317260742,
        0.20191457867622375,
        -2.5927578258514404,
        0.538416862487793,
        2.7445449829101562,
        1.5043296813964844,
        1.7523150444030762,
        0.0
    ],
    "left": [
        -1.46564781665802,
        0.3302380442619324,
        2.507143497467041,
        0.7764986753463745,
        -2.852548837661743,
        1.659092664718628,
        1.378138542175293,
        0.0
    ]
}

CALIB = {
    "left": [0.0, -2.2699, 2.3561, 0.5224, 0.0033, 0.6971, 0.0035, 0.0],
    "right": [0.0042, -2.2704, -2.353, 0.5187, -0.0048, 0.6965, 0.0051, 0.0]
    # 'right':[0.2838688057286251, -1.8515421707929456, -1.2710114427746415, 0.9488596708826394, 0.38491125769127615, 0.09287151314189822, 0.661390729060855, 0.0]
}

def check_join_target(value):
    """check joint value"""
    return value <= pi and value >= -pi

def clamp(num, min_value, max_value):
    """Clamp"""
    return max(min(num, max_value), min_value)

def get_orientation_from_lego_orientation(horizontal=True):
    return [pi, 0, 0] if horizontal else [pi, 0, pi/2]

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
    # pose_target.orientation.x = -0.8429934558409764
    # pose_target.orientation.y =  -0.11190617985736794
    # pose_target.orientation.z = -0.5217104399067845
    # pose_target.orientation.w = 0.06824410019399091

    return pose_target

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



class Yumi:
    """
        Yumi
    """

    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)
        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        self.robot = RobotCommander()
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

    def changeEGMSettings(self):
        """Changes EGM Settings"""
        rospy.wait_for_service('/yumi/rws/sm_addin/get_egm_settings')
        rospy.wait_for_service('/yumi/rws/sm_addin/set_egm_settings')
        # rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")

        # stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)

        get_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/get_egm_settings", GetEGMSettings)
        set_egm_settings = rospy.ServiceProxy("/yumi/rws/sm_addin/set_egm_settings", SetEGMSettings)

        current_settings_L = get_egm_settings(task='T_ROB_L')
        current_settings_R = get_egm_settings(task='T_ROB_R')

        settings_L = current_settings_L.settings
        settings_R = current_settings_R.settings

        # max_speed_deviation is in deg/s, we convert from rad/s
        settings_L.activate.max_speed_deviation = degrees(7.0)
        settings_R.activate.max_speed_deviation = degrees(7.0)

        # settings.activate.cond_min_max

        # settings.run.cond_time
        settings_L.run.cond_time = 60.0
        settings_R.run.cond_time = 60.0
        # posCorrgain = 0.0
        settings_L.run.pos_corr_gain = 0.0
        settings_R.run.pos_corr_gain = 0.0

        # stop_egm()
        taskname = "T_ROB_L"
        set_egm_settings(task=taskname, settings=settings_L)

        taskname = "T_ROB_R"
        set_egm_settings(task=taskname, settings=settings_R)

        print("===================== EGM Settings Updated =====================")

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

        # TODO: check pose or joint egm
        rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
        rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
        # rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

        start_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
        stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)

        if (right):
            arm = 'right'
            group = self.group_r
            controller = "right_arm_vel_controller"
        else:
            arm = 'left'
            group = self.group_l
            controller ="left_arm_vel_controller"

        rospy.loginfo('Planning and moving ' + arm + ' arm: Position: {' + str(target.position.x) + ';' + str(target.position.y) + ';' + str(target.position.z) +
                                            '}. Rotation: {' + str(euler[0]) + ';' + str(euler[1]) + ';' + str(euler[2]) + '}.')

        tic = time.time()

        group.set_pose_target(copy.deepcopy(target))
        plan = group.plan()
        # waypoints = []

        # wpose = group.get_current_pose().pose
        # print(wpose)
        # waypoints.append(copy.deepcopy(wpose))
        # waypoints.append(copy.deepcopy(target))
        # (plan, fraction) = group.compute_cartesian_path(
        #     waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        # )  # jump_threshold
        for p_ in plan:
            print('///////////////////////////////////////////////////////////////')
            print(type(p_))
            print(p_)
        # print(fraction)
        toc = time.time()
        dur1 = round(toc - tic, 3)
        print("\nPlanning Time is: {} seconds!".format(dur1))
        time.sleep(1)
        self.changeEGMSettings()
        start_egm()

        controller_conf = "start_controllers: [{}] \nstop_controllers: [''] \nstrictness: 1 \nstart_asap: false \ntimeout: 0.0".format(controller)
        import subprocess
        subprocess.run(["rosservice", "call", "/yumi/egm/controller_manager/switch_controller", controller_conf])

        # success = group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement

        # group.execute(plan_x, wait=True)
        # group.execute(plan_y, wait=True)
        # group.execute(plan_z, wait=True)
        group.go(wait=True)
        group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        group.clear_pose_targets()
        rospy.sleep(3)

    def go_to_joint_state(self, joint_goal, group, controller):
        """Plans and moves the selected group to the joint goal

        Creates a plan to move a group to the given joint goal.

        @param target: The joint goal
        @param group: The group to move
        @controller: The controller
        @returns None
        """

        rospy.wait_for_service("/yumi/rws/sm_addin/start_egm_joint")
        rospy.wait_for_service("/yumi/rws/sm_addin/stop_egm")
        # rospy.wait_for_service("/yumi/egm/controller_manager/switch_controller")

        start_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/start_egm_joint", TriggerWithResultCode)
        stop_egm = rospy.ServiceProxy("/yumi/rws/sm_addin/stop_egm", TriggerWithResultCode)
        # switch_controller = rospy.ServiceProxy("/yumi/egm/controller_manager/switch_controller", SwitchController)

        stop_egm()

        tic = time.time()

        group.set_joint_value_target(joint_goal)
        group.plan()

        toc = time.time()
        dur1 = round(toc - tic, 3)
        print("\nPlanning Time is: {} seconds!".format(dur1))
        # TODO: check if this sleep is necessary.
        time.sleep(1)
        self.changeEGMSettings()
        start_egm()

        # print("controller:", controller)

        # switch_controller(start_controllers=list(controller), stop_controllers=[""], strictness=1, start_asap=False, timeout=0.0)
        controller_conf = "start_controllers: [{}] \nstop_controllers: [''] \nstrictness: 1 \nstart_asap: false \ntimeout: 0.0".format(controller)
        import subprocess
        subprocess.run(["rosservice", "call", "/yumi/egm/controller_manager/switch_controller", controller_conf])

        group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        group.stop()

        rospy.sleep(3)

    def go_to_joint_state_arm(self, joint_goal, right=True):
        """Plans and moves the selected arm to the joint goal

        Creates a plan to move a group to the given joint goal.

        @param target: The joint goal
        @param right True if the selected arm is the right one
        @returns None
        """

        group = self.group_r if right else self.group_l
        controller = "right_arm_vel_controller" if right else "left_arm_vel_controller"
        self.go_to_joint_state(joint_goal, group, controller)

    def go_to_joint_state_both(self, joint_goal):
        """Plans and moves both arms to the joint goal

        Creates a plan to move a group to the given joint goal.

        @param right True if the selected arm is the right one
        @returns None
        """

        self.go_to_joint_state(joint_goal, self.group_both, "both_arms_vel_controller")

    ###################################################
    ############### Grippers Methods ##################
    ###################################################

    def calibrateGrippers(self):
        """Calibrates Yumi's Gripper"""
        rospy.wait_for_service('/yumi/rws/sm_addin/set_sg_command')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_sg_routine')

        set_sg_command = rospy.ServiceProxy("/yumi/rws/sm_addin/set_sg_command", SetSGCommand)
        run_sg_routine = rospy.ServiceProxy("/yumi/rws/sm_addin/run_sg_routine", TriggerWithResultCode)

        """
        uint8 SG_COMMAND_INITIALIZE   = 3
        uint8 SG_COMMAND_CALIBRATE    = 4
        uint8 SG_COMMAND_MOVE_TO      = 5
        uint8 SG_COMMAND_GRIP_IN      = 6
        uint8 SG_COMMAND_GRIP_OUT     = 7
        """
        task_l = 'T_ROB_L'
        task_r = 'T_ROB_R'
        fin_pos = 0.0
        # # Close Grippers
        cmd = 6
        set_sg_command(task=task_l, command=cmd, target_position=fin_pos)
        set_sg_command(task=task_r, command=cmd, target_position=fin_pos)
        run_sg_routine()
        time.sleep(1)
        # # Calibrate Grippers
        cmd = 4
        set_sg_command(task=task_l, command=cmd, target_position=fin_pos)
        set_sg_command(task=task_r, command=cmd, target_position=fin_pos)
        run_sg_routine()
        time.sleep(2)
        print('===================== Grippers: Calibrated =====================')


    def effort(self, pos, right=True):
        """Set gripper effort

        Sends a pos command to the selected gripper. Should be in the range of
        20.0 (fully open) to 0.0 (fully closed)

        :param gripper_id: The ID of the selected gripper (LEFT or RIGHT)
        :param pos: The pos value for the gripper (0.0 to 20.0)
        :type gripper_id: int
        :type pos: float
        :returns: Nothing
        :rtype: None
        """
        pos = clamp(pos, 0, 20)
        gripper_id = 'right' if right else 'left'
        rospy.loginfo("Setting gripper " + str(gripper_id) + " to " + str(pos))
        rospy.loginfo('Setting gripper pos to ' + str(pos) + ' for arm ' + str(gripper_id))

        rospy.wait_for_service('/yumi/rws/sm_addin/set_sg_command')
        rospy.wait_for_service('/yumi/rws/sm_addin/run_sg_routine')

        set_sg_command = rospy.ServiceProxy("/yumi/rws/sm_addin/set_sg_command", SetSGCommand)
        run_sg_routine = rospy.ServiceProxy("/yumi/rws/sm_addin/run_sg_routine", TriggerWithResultCode)

        task_L = 'T_ROB_L'
        task_R = 'T_ROB_R'
        cmd = 5
        if right:
            set_sg_command(task=task_R, command=cmd, target_position=pos)
            run_sg_routine()
            time.sleep(1)
        else:
            set_sg_command(task=task_L, command=cmd, target_position=pos)
            run_sg_routine()
            time.sleep(1)
        # elif gripper_id == BOTH:
        #     set_sg_command(task=task_L, command=cmd, target_position=pos)
        #     set_sg_command(task=task_R, command=cmd, target_position=pos)
        #     run_sg_routine()
        #     time.sleep(1)

        rospy.sleep(1.0)
# jkh

class MoveItInterface(object):
    """MoveItInterface"""

    def __init__(self):
        super(MoveItInterface, self).__init__()
        self.sys_init()
        self.motion_init()


    def print_status(self):
        """Print Status"""
        self.yumi.print_status()

    def sys_init(self):
        """
        Initialize the system, auto_mode, motors_on, rapid_running,
        egm_settings_adjusted
        """
        auto_mode = True # system_info.auto_mode #
        # rapid_running = system_info.rapid_running
        motors_on = False # system_info.motors_on #
        # print(auto_mode, motors_on)

        if(not auto_mode):
            print("xxxxxxxxxxxxx Robot in Manual Mode! Can't Initialize! xxxxxxxxxxxxx")
            sys.exit()
        else:
            # Reset Program Pointer to main and Start RAPID
            rospy.wait_for_service('/yumi/rws/pp_to_main')
            rospy.wait_for_service('/yumi/rws/stop_rapid')
            rospy.wait_for_service('/yumi/rws/start_rapid')
            # set_motors_off= rospy.ServiceProxy("/yumi/rws/set_motors_off",TriggerWithResultCode)
            self.stop_rapid    = rospy.ServiceProxy("/yumi/rws/stop_rapid"   , TriggerWithResultCode)
            self.pp_to_main    = rospy.ServiceProxy("/yumi/rws/pp_to_main"   , TriggerWithResultCode)
            self.start_rapid   = rospy.ServiceProxy("/yumi/rws/start_rapid"  , TriggerWithResultCode)
            # Call the functionalities
            self.stop_rapid()
            self.pp_to_main()
            time.sleep(1)
            self.start_rapid()
            # Turn Motors On
            if(not motors_on):
                rospy.wait_for_service('/yumi/rws/set_motors_on')
                self.set_motors_on = rospy.ServiceProxy("/yumi/rws/set_motors_on", TriggerWithResultCode)
                self.set_motors_on()

    def motion_init(self):
        """Init motion interface"""
        self.scene = PlanningSceneInterface()
        rospy.init_node("moveit_interface", anonymous=True)
        print("============ Node moveit_interface Initialized")
        rospy.sleep(2)

        self.yumi = Yumi()
        self.add_scene_table()
        self.add_block(
            Block(0,[4,4],[5,4],[5,5],[4,5])
        )
        self.add_block(
            Block(1,[24,11],[25,11],[25,12],[24,12])
        )
        self.add_block(
            Block(2,[45,23],[48,23],[48,24],[45,24])
        )
        rospy.sleep(3)
        self.calibrateGrippers()
        # self.go_to_idle()


    def add_block(self, block):
        """Add block in the scene"""
        block_name = f"block_{block.id}"
        self.scene.remove_world_object(block_name)
        # self.scene.add_box(
        #     block_name,
        #     block.get_real_position(self.yumi),
        #     block.get_dimension())

    def add_scene_table(self):
        """Add a table on the scene"""
        self.scene.remove_world_object("table")
        # self.add_scene_box()
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
        # self.scene.add_box("table", p, (0.510, 1.000, 0.001))

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
        # goal_radians = []
        # # goal_radians.extend(CALIB["left"])
        # # goal_radians.extend(CALIB["right"])
        # goal_radians.extend(IDLE["left"])
        # goal_radians.extend(IDLE["right"])
        self.yumi.go_to_joint_state_arm(IDLE["left"], right=False)
        self.yumi.go_to_joint_state_arm(IDLE["right"], right=True)
        self.print_status()

    def go_to_calib(self):
        """ Moves yumi to its idle position"""
        # goal_radians = []
        # goal_radians.extend(CALIB["left"])
        # goal_radians.extend(CALIB["right"])
        self.yumi.go_to_joint_state_arm(CALIB["left"], right=False)
        self.yumi.go_to_joint_state_arm(CALIB["right"], right=True)
        self.print_status()

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
                    interface.go_to_calib()
                    break
            except e:
                print("ERROR")
                print('End of the session')
                interface.go_to_calib()
                break
    except KeyboardInterrupt:
        print('End of the session')
        interface.go_to_idle()



if __name__ == "__main__":
    main()
