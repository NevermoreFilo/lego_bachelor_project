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
    def __init__(self):

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
        self.start_joint_goal = [0.0, -0.785398163397, 0.0, -2.35619449019, 0.0, 1.57079632679,
                                 0.785398163397]  # Posizione a riposo del robot

        # Dimensione dei lego
        self.legoWidth = 0.0314
        self.legoHeight = 0.019

        self.openOffSet = 0.009  # Offset da aggiungere all'apertura del gripper oltre alla larghezza/lunghezza del lego
        self.closedOffset = 0.005  # Offset da aggiungere alla chiusura del gripper (la dimensionee del lego non è sufficiente)
        self.eef_height = 0.111  # Offset sull'asse delle z, preso dalla documentazione ufficiale
        self.upOffset = 0.015 # Offset a cui il braccio si alza dopo aver piazzato il pezzo

        # Inizializzazione liste di coordinate
        self.starting_longP_coordinates = []
        self.starting_longP_aux_coords = [[-0.08022024347283004, 0.4345996311647573, -0.017687250096303953],
                                          [-0.077125263174171483, 0.6157033161972792, -0.015843953334895085],
                                          [0.02097427655921029, 0.434620359716768, -0.01736008183053387],
                                          [0.021374736824171483, 0.6157033161972792, -0.015843953334895085],
                                          [0.12239073295454152, 0.4347217843628824, -0.016965818155016486],
                                          [0.4201145557236046, 0.44608137254548264, -0.014633344607714552]]

        self.starting_shortP_coordinates = []
        self.starting_shortP_aux_coords = [[0.21788618069731605, 0.3082757604473198, -0.018247824801653288],
                                           [-0.22313862587754232, 0.43369528155501386, -0.017351559286898043],
                                           [0.32142728885822247, 0.3071422312992683, -0.017243796812094654],
                                           [0.3226593631314784, 0.43208637933142, -0.015421032728421769]]

        self.place_short_coordinates = []
        self.place_short_aux_coords = [[0.20288832488009007, 0.5854943504460622, 0.01264770456857292]]

    # Funzione per l'inizializzazione delle liste di coordinate
    def init_coordinates(self):
        for index in range(6):
            self.starting_longP_coordinates.append(Coordinate(self.starting_longP_aux_coords[index][0],
                                                              self.starting_longP_aux_coords[index][1],
                                                              self.starting_longP_aux_coords[index][2]))
        for index in range(4):
            self.starting_shortP_coordinates.append(Coordinate(self.starting_shortP_aux_coords[index][0],
                                                               self.starting_shortP_aux_coords[index][1],
                                                               self.starting_shortP_aux_coords[index][2]))

        for index in range(1):
            self.place_short_coordinates.append(Coordinate(self.place_short_aux_coords[index][0],
                                                           self.place_short_aux_coords[index][1],
                                                           self.place_short_aux_coords[index][2]))

    def go_to_starting_pose(self):
        self.move_group.go(self.start_joint_goal, wait=True)

    def go_to_working_pose(self):
        # self.move_group.set_max_velocity_scaling_factor(1)
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[0] = self.start_joint_goal[0] + math.pi / 2
        joint_goal[1] = self.start_joint_goal[1]
        joint_goal[2] = self.start_joint_goal[2]
        joint_goal[3] = self.start_joint_goal[3]
        joint_goal[4] = self.start_joint_goal[4]
        joint_goal[5] = self.start_joint_goal[5]
        joint_goal[6] = self.start_joint_goal[6] + math.pi / 2
        self.move_group.go(joint_goal, wait=True)

    # Funzione per portare il gripper nella posa del primo grip (quella dal lato corto)
    def pre_grasp_rotation(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[6] = joint_goal[6] - math.pi / 2
        self.move_group.go(joint_goal, wait=True)

    # Funzione per portare il gripper nella posa del primo grip (quella dal lato lungo)
    def grasp_rotation(self):
        joint_goal = self.move_group.get_current_joint_values()
        joint_goal[6] = joint_goal[6] + math.pi / 2
        self.move_group.go(joint_goal, wait=True)

    def upward_retreat(self):
        waypoints = []
        wpose = self.move_group.get_current_pose().pose
        wpose.position.z = wpose.position.z + self.upOffset
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                            velocity_scaling_factor=0.8, acceleration_scaling_factor=0.03
                                            )  # Per rallentare il robot
        self.move_group.execute(plan, wait=True)

    def pick(self, coordinate):
        waypoints = []
        self.pre_grasp_rotation()
        wpose = self.move_group.get_current_pose().pose
        wpose.position.x = coordinate.x
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.y = coordinate.y
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z = coordinate.z + ((self.legoHeight * 2) / 3) + self.eef_height # Il grasp avviene a 2/3 circa dell'altezza del lego
        waypoints.append(copy.deepcopy(wpose))
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = self.move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                            velocity_scaling_factor=0.8, acceleration_scaling_factor=0.03
                                            )  # Per rallentare il robot
        self.move_group.execute(plan, wait=True)
        self.close_gripper()
        rospy.sleep(0.5)
        self.open_gripper()
        rospy.sleep(0.5)
        self.grasp_rotation()
        wpose.position.z = coordinate.z + ((self.legoHeight * 2) / 3) + self.eef_height # Il grasp avviene a 2/3 circa dell'altezza del lego
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,
            0.0
        )
        plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                            velocity_scaling_factor=0.6, acceleration_scaling_factor=0.03
                                            )  # Per rallentare il robot
        move_group.execute(plan, wait=True)
        print("Sono tornato giu. richiudo il gripper")
        close_gripper()

robot = Robot()
robot.init_coordinates()
print(robot.starting_longP_coordinates[0].x)
