#!/usr/bin/env python

import copy
# Author: Ioan Sucan, Ridhwan Luthra
# Contributor: Rick Staa (Translated code from cpp to python)

## Python standard library imports ##
import sys
import math

import moveit_msgs
## Ros ##
import rospy

## MoveIt and TF ##
import moveit_commander
from control_msgs.msg import GripperCommand
from tf.transformations import quaternion_from_euler

## Import msgs and srvs ##
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectoryPoint
from moveit_msgs.msg import DisplayTrajectory, Grasp, PlaceLocation
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneRequest, GetPlanningScene

legoWidth = 0.04
openOffSet = 0
closedOffset = 0.03
x_start = 0.5
y_start = 0
z_start = 0

x_end = 0.6
y_end = 0
z_end = 0.1
def openGripper(posture):
    ## - BEGIN_SUB_TUTORIAL open_gripper- ##
    ## Add both finger joints of panda robot ##
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    ## Set them as open, wide enough for the object to fit. ##
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.02
    posture.points[0].positions[1] = 0.02
    posture.points[0].time_from_start = rospy.Duration(0.5)
    print("opened")
    ## - END_SUB_TUTORIAL - ##


def closedGripper(posture):
    ## - BEGIN_SUB_TUTORIAL open_gripper - ##
    ## Add both finger joints of panda robot. ##
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    ## Set them as closed. ##
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = 0.00
    posture.points[0].positions[1] = 0.00
    posture.points[0].time_from_start = rospy.Duration(0.5)
    print("closed")
    ## - END_SUB_TUTORIAL - ##


def pick(move_group):
    waypoints = []
    #scale = 1.0
    wpose = move_group.get_current_pose().pose
    wpose.position.x = x_start
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z = z_start + 0.105
    # wpose.position.z = z_start - 0.035
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )
    move_group.execute(plan, wait=True)
    #planning_scene_interface.attach_box(eef_link, 'object')
    grasps = Grasp()
    grasps.grasp_pose.header.frame_id = "panda_link0"
    grasps.grasp_pose.pose.orientation.x = move_group.get_current_pose().pose.orientation.x
    grasps.grasp_pose.pose.orientation.y = move_group.get_current_pose().pose.orientation.y
    grasps.grasp_pose.pose.orientation.z = move_group.get_current_pose().pose.orientation.z
    grasps.grasp_pose.pose.orientation.w = move_group.get_current_pose().pose.orientation.w

    grasps.grasp_pose.pose.position.x = move_group.get_current_pose().pose.position.x
    grasps.grasp_pose.pose.position.y = move_group.get_current_pose().pose.position.y
    grasps.grasp_pose.pose.position.z = move_group.get_current_pose().pose.position.z

    grasps.pre_grasp_approach.direction.header.frame_id = "panda_link0"
    grasps.pre_grasp_approach.direction.vector.x = 0.1
    grasps.pre_grasp_approach.min_distance = 0
    grasps.pre_grasp_approach.desired_distance = 0.001

    grasps.post_grasp_retreat.direction.header.frame_id = "panda_link0"
    grasps.post_grasp_retreat.direction.vector.z = 1.0
    grasps.post_grasp_retreat.min_distance = 0.1
    grasps.post_grasp_retreat.desired_distance = 0.25

    closedGripper(grasps.grasp_posture)
    #move_group.attach_object('object')
    move_group.set_support_surface_name("table")
    move_group.pick("object", grasps)


def place(group):
    waypoints = []
    #scale = 1.0
    wpose = move_group.get_current_pose().pose
    wpose.position.x = x_end
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y = y_end
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z = z_start + 0.105
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )
    move_group.execute(plan, wait=True)
def allow_contact(obj):
    current_scene = get_planning_scene()
    scene_diff = ApplyPlanningSceneRequest()
    scene_diff.scene.is_diff = True
    acm = current_scene.scene.allowed_collision_matrix
    allowed_collision_links = ['panda_hand', 'panda_rightfinger', 'panda_leftfinger', 'panda_hand_sc', 'panda_link1_sc']
    if not obj in acm.entry_names:
        values = [name in allowed_collision_links for name in acm.entry_names]
        for entry, val in zip(acm.entry_values, values):
            entry.enabled.append(val)
        acm.entry_names.append(obj)
        entry = moveit_msgs.msg.AllowedCollisionEntry()
        entry.enabled = values + [False]
        acm.entry_values.append(entry)
    else:
        ind = acm.entry_names.index(obj)
        for entry, name in zip(acm.entry_values, acm.entry_names):
            entry.enabled[ind] = name in allowed_collision_links
    scene_diff.scene.allowed_collision_matrix = acm
    set_planning_scene(scene_diff)


def addCollisionObjects(planning_scene_interface):
    collision_objects_names = [str for i in range(2)]
    collision_object_sizes = [str for i in range(2)]
    collision_objects = [PoseStamped() for i in range(2)]

    ## Define the object that we will be manipulating ##
    collision_objects_names[0] = "object"
    collision_objects[0].header.frame_id = "panda_link0"

    ## Define the primitive and its dimensions. ##
    collision_object_sizes[0] = (0.032, 0.032, 0.024)  # Box size

    ## Define the pose of the object. ##
    collision_objects[0].pose.position.x = x_start
    collision_objects[0].pose.position.y = y_start
    collision_objects[0].pose.position.z = z_start

    ## Define the table ##
    collision_objects_names[1] = "table"
    collision_objects[1].header.frame_id = "panda_link0"
    collision_object_sizes[1] = (1, 1, 0.2)
    collision_objects[1].pose.position.x = 0
    collision_objects[1].pose.position.y = 0
    collision_objects[1].pose.position.z = -0.12

    ## Add collision objects to scene ##

    for (name, pose, size) in zip(
            collision_objects_names, collision_objects, collision_object_sizes
    ):
        planning_scene_interface.add_box(name=name, pose=pose, size=size)

    #planning_scene_interface.add_box(name=collision_objects_names[0], pose=collision_objects[0], size=collision_object_sizes[0])
    allow_contact('object')
    allow_contact('table')

if __name__ == "__main__":

    ## Initialize ros node ##
    rospy.init_node("panda_arm_pick_place")

    ## initialize moveit_commander ##
    moveit_commander.roscpp_initialize(sys.argv)

    ## Connect to moveit services ##
    rospy.loginfo(
        "Conneting moveit default moveit 'apply_planning_scene' service.")
    rospy.wait_for_service("apply_planning_scene")
    try:
        planning_scene_srv = rospy.ServiceProxy(
            "apply_planning_scene", ApplyPlanningScene
        )
        rospy.loginfo("Moveit 'apply_planning_scene' service found!")
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

    ## Create robot commander ##
    robot = moveit_commander.RobotCommander(
        robot_description="robot_description", ns="/"
    )
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    set_planning_scene=rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    rospy.logdebug("Robot Groups: %s", robot.get_group_names())

    ## Create scene commanders ##
    # Used to get information about the world and update the robot
    # its understanding of the world.
    move_group = robot.get_group("panda_arm")
    planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")
    ## Specify the planner we want to use ##
    move_group.set_planner_id("TRRTkConfigDefault")

    ## Create a `DisplayTrajectory`_ ROS publisher to display the plan in RVIZ ##
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )
    eef_link = move_group.get_end_effector_link()

    ## Wait a bit for ROS to initialize the planning_scene_interface ##
    rospy.sleep(1.0)

    ## Add collision objects ##
    addCollisionObjects(planning_scene_interface)

    ## Wait a bit ##
    rospy.sleep(1.0)

    ## Pick ##
    pick(move_group)

    ## Wait a bit ##
    rospy.sleep(1.0)

    ## Place ##
   # place(move_group)
