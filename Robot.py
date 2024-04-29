#!/usr/bin/env python

import copy
# Author: Ioan Sucan, Ridhwan Luthra
# Contributor: Rick Staa (Translated code from cpp to python)

## Python standard library imports ##
import sys
import numpy as np
import math

import actionlib
import franka_gripper
import geometry_msgs.msg
import moveit_msgs
## Ros ##
import rospy

## MoveIt and TF ##
import moveit_commander
from control_msgs.msg import GripperCommand, GripperCommandAction
from franka_gripper.msg import MoveAction, GraspAction
from tf.transformations import quaternion_from_euler

## Import msgs and srvs ##
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, Grasp, PlaceLocation, Constraints, OrientationConstraint
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest, GetPlanningScene


class Coordinate():
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z


class Robot():
    def __init(self):

        # Inizializzazione generale

        rospy.init_node("panda_arm_lego_pick_place")  # Inizializzazione nodo ROS
        moveit_commander.roscpp_initialize(sys.argv)  # Inizializzazione moveit_commander
        rospy.wait_for_service("apply_planning_scene")
        try:
            planning_scene_srv = rospy.ServiceProxy(
                "apply_planning_scene", ApplyPlanningScene
            )
        except rospy.ServiceException as e:
            rospy.logerr(
                "Moveit 'apply_planning_scene' service initialization failed: %s" % e
            )
            shutdown_msg = "Shutting down %s node because %s service connection failed." % (
                rospy.get_name(),
                planning_scene_srv.resolved_name,
            )
            rospy.logerr(shutdown_msg)
            sys.exit(0)

        self.get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
        self.set_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
        self.move_group = moveit_commander.MoveGroupCommander("panda_arm")
        self.planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")
        self.move_group.set_planner_id("TRRT")  # Specifico il planner utilizzato
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
        )
        self.start_joint_goal = [0.0, -0.785398163397, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.785398163397]

        self.legoWidth = 0.0314
        self.legoHeight = 0.019
        self.openOffSet = 0.009
        self.closedOffset = 0.005
        self.starting_longP_coordinates = []
        self.starting_longP_aux_coords = [[-0.08022024347283004, 0.4345996311647573, -0.017687250096303953],
                                          [-0.077125263174171483, 0.6157033161972792, -0.015843953334895085],
                                          [0.02097427655921029, 0.434620359716768, -0.01736008183053387],
                                          [0.021374736824171483, 0.6157033161972792, -0.015843953334895085],
                                          [0.12239073295454152, 0.4347217843628824, -0.016965818155016486],
                                          [0.4201145557236046, 0.44608137254548264, -0.014633344607714552]]

    def init_coordinates(self):
        for index in range(6):
            self.starting_longP_coordinates.append(Coordinate(self.starting_longP_aux_coords[index][0],
                                                              self.starting_longP_aux_coords[index][1],
                                                              self.starting_longP_aux_coords[index][2]))


robot = Robot()
print(robot.legoHeight)
#robot.init_coordinates()
#print(robot.starting_longP_coordinates.x)
