#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from __future__ import print_function

""" A moveit interface to control Yumi."""

__author__ = "Maxence Grand"
__copyright__ = "TODO"
__credits__ = ["TODO"]
__license__ = "TODO"
__version__ = "0.0.1"
__maintainer__ = "Maxence Grand"
__email__ = "Maxence.Grand@univ-grenoble-alpes.fr"
__status__ = "Under Developpement"

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
