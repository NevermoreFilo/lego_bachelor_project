#!/usr/bin/env python

import copy
# Author: Ioan Sucan, Ridhwan Luthra
# Contributor: Rick Staa (Translated code from cpp to python)

## Python standard library imports ##
import sys
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

legoWidth = 0.0314
legoHeight = 0.019
openOffSet = 0.012
closedOffset = 0.005
"""
Primo punto
x_start = 0.22313862587754232
y_start = 0.43369528155501386
z_start = -0.017351559286898043

#1
x_start = 0.07046725964075157
y_start = 0.43477082193286887
z_start = -0.017275654103103416
#2
x_start = 0.12239073295454152
y_start = 0.4347217843628824
z_start = -0.016965818155016486

#3
x_start = 0.1715555070842264
y_start = 0.4339485685723369
z_start = -0.016552528203041164

#4
x_start = 0.2712850601943779
y_start = 0.4325605336689113
z_start = -0.015645227604258682

#5
x_start = 0.3226593631314784
y_start = 0.43208637933142
z_start = -0.015421032728421769

#6
x_start = 0.3705750262848844
y_start = 0.4481066976926753
z_start = -0.014774046909519792

#7
x_start = 0.4201145557236046
y_start = 0.44608137254548264
z_start = -0.014633344607714552

#8
x_start = 0.0718341463745228
y_start = 0.6146257537422227
z_start = -0.0155668052647003

#9
x_start = 0.1250581629718011
y_start = 0.6138304330190797
z_start = -0.015209999247854773

#d1
x_end = 0.20288832488009007
y_end = 0.5844943504460622
z_end = 0.01264770456857292

#d2
x_end = 0.2837040663196856
y_end = 0.5988946726759686
z_end = 0.013544381106687417

#d4
x_end = 0.3958770521169853
y_end = 0.5965657314936886
z_end = 0.014568227926752192

#d5
x_end = 0.3953770521169853 + legoWidth
y_end = 0.5965657314936886
z_end = 0.013568227926752192



"""
#1
x_start = 0.07046725964075157
y_start = 0.43477082193286887
z_start = -0.017275654103103416

#d3
x_end = 0.2837040663196856 + legoWidth
y_end = 0.5988946726759686
z_end = 0.013544381106687417

eef_height = 0.111

"""
def openGripper(posture):
    ## - BEGIN_SUB_TUTORIAL open_gripper- ##
    ## Add both finger joints of panda robot ##
    posture.joint_names = [str for i in range(2)]
    posture.joint_names[0] = "panda_finger_joint1"
    posture.joint_names[1] = "panda_finger_joint2"

    ## Set them as open, wide enough for the object to fit. ##
    posture.points = [JointTrajectoryPoint()]
    posture.points[0].positions = [float for i in range(2)]
    posture.points[0].positions[0] = legoWidth + openOffSet
    posture.points[0].positions[1] = legoWidth + openOffSet
    posture.points[0].time_from_start = rospy.Duration(0.5)
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
    posture.points[0].positions[0] = legoWidth - closedOffset
    posture.points[0].positions[1] = legoWidth - closedOffset
    posture.points[0].time_from_start = rospy.Duration(0.5)
    ## - END_SUB_TUTORIAL - ##
"""
    
def go_to_neutral_pose(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = start_joint_goal[0]
    joint_goal[1] = start_joint_goal[1]
    joint_goal[2] = start_joint_goal[2]
    joint_goal[3] = start_joint_goal[3]
    joint_goal[4] = start_joint_goal[4]
    joint_goal[5] = start_joint_goal[5]
    joint_goal[6] = start_joint_goal[6]
    move_group.go(joint_goal, wait=True)



def go_to_starting_pose(move_group):
    move_group.set_max_velocity_scaling_factor(0.5)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = start_joint_goal[0] + math.pi/2
    joint_goal[1] = start_joint_goal[1]
    joint_goal[2] = start_joint_goal[2]
    joint_goal[3] = start_joint_goal[3]
    joint_goal[4] = start_joint_goal[4]
    joint_goal[5] = start_joint_goal[5]
    joint_goal[6] = start_joint_goal[6] + math.pi/2
    move_group.go(joint_goal, wait=True)

def rotate_gripper1(move_group):
    print("Ruoto il gripper")
    joint_goal = move_group.get_current_joint_values()
    joint_goal[6] = joint_goal[6] - math.pi /2
    move_group.go(joint_goal, wait=True)

def rotate_gripper2(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[6] = joint_goal[6] + math.pi /2
    move_group.go(joint_goal, wait=True)

def go_to_neutral_pose2(move_group):
    waypoints = []
    # scale = 1.0
    wpose = move_group.get_current_pose().pose
    wpose.position.z = wpose.position.z + 0.10
    waypoints.append(copy.deepcopy(wpose))
    # wpose.position.z = z_end + 0.105 + 0.01

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )
    plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                        velocity_scaling_factor=0.09, acceleration_scaling_factor=0.02
                                        )  # Per rallentare il robot
    move_group.execute(plan, wait=True)


def pick(move_group):
    waypoints = []
    # scale = 1.0

    rotate_gripper1(move_group)
    wpose = move_group.get_current_pose().pose
    wpose.position.x = x_start
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y = y_start
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z = z_start + ((legoHeight*2)/3) + eef_height
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )
    plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                        velocity_scaling_factor=0.09, acceleration_scaling_factor=0.02
                                        )  # Per rallentare il robot
    move_group.execute(plan, wait=True)
    close_gripper()
    rospy.sleep(0.5)
    open_gripper()
    rospy.sleep(0.5)
    rotate_gripper2(move_group)
    wpose.position.z = z_start + ((legoHeight * 2) / 3) + eef_height
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )
    plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                        velocity_scaling_factor=0.08, acceleration_scaling_factor=0.02
                                        )  # Per rallentare il robot
    move_group.execute(plan, wait=True)
    print("Sono tornato giu. richiudo il gripper")
    close_gripper()
    # planning_scene_interface.attach_box(eef_link, 'object')
    """
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
    grasps.pre_grasp_approach.direction.vector.x = 0.0
    grasps.pre_grasp_approach.direction.vector.y = 0.0
    grasps.pre_grasp_approach.direction.vector.z = 1
    grasps.pre_grasp_approach.min_distance = 0.0
    grasps.pre_grasp_approach.desired_distance = 0.01

    grasps.post_grasp_retreat.direction.header.frame_id = "panda_link0"
    grasps.post_grasp_retreat.direction.vector.x = 0.0
    grasps.post_grasp_retreat.direction.vector.x = 0.0
    grasps.post_grasp_retreat.direction.vector.z = 1
    grasps.post_grasp_retreat.min_distance = 0.0
    grasps.post_grasp_retreat.desired_distance = 0.01
    openGripper(grasps.pre_grasp_posture)
    closedGripper(grasps.grasp_posture)
    move_group.set_support_surface_name("table")
    print(move_group.get_current_pose().pose.position.x)
    print(move_group.get_current_pose().pose.position.y)
    print(move_group.get_current_pose().pose.position.z)
   # move_group.pick("object", grasps)
    print("")
    print(move_group.get_current_pose().pose.position.x)
    print(move_group.get_current_pose().pose.position.y)
    print(move_group.get_current_pose().pose.position.z)
    """


def open_gripper():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.

    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal(width=2*legoWidth+openOffSet, speed=0.02)
    # goal.width = 0.022
    # goal.speed = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    print("opened")
    return client.get_result()  # A move result

def open_gripper2():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/franka_gripper/move', franka_gripper.msg.MoveAction)

    # Waits until the action server has started up and started
    # listening for goals.

    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.MoveGoal(width=legoWidth+openOffSet, speed=0.02)
    # goal.width = 0.022
    # goal.speed = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    print("opened")
    return client.get_result()  # A move result


def close_gripper():
    # Creates the SimpleActionClient, passing the type of the action
    client = actionlib.SimpleActionClient('/franka_gripper/grasp', franka_gripper.msg.GraspAction)

    # Waits until the action server has started up and started
    # listening for goals.

    client.wait_for_server()

    # Creates a goal to send to the action server.
    goal = franka_gripper.msg.GraspGoal(width= 2*legoWidth - closedOffset, speed=0.02, force=5)
    goal.epsilon.inner = 0.01
    goal.epsilon.outer = 0.01
    # goal.width = 0.022
    # goal.speed = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()  # A move result


def place(group):
    print("sto provando a piazzare")
    waypoints = []
    # scale = 1.0
    wpose = move_group.get_current_pose().pose
    #wpose.position.x = x_end - 0.002
    wpose.position.x = x_end
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.y = y_end - 0.001
    waypoints.append(copy.deepcopy(wpose))
    wpose.position.z = z_end + eef_height - 0.0075
    waypoints.append(copy.deepcopy(wpose))

    (plan, fraction) = move_group.compute_cartesian_path(
        waypoints,
        0.01,
        0.0
    )
    plan = move_group.retime_trajectory(moveit_commander.RobotCommander().get_current_state(), plan,
                                        velocity_scaling_factor=0.07, acceleration_scaling_factor=0.01)  # Per rallentare il robot
    move_group.execute(plan, wait=True)
    open_gripper()
    # planning_scene_interface.remove_attached_object('object')


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
    collision_objects[0].pose.position.x = x_start + 1
    collision_objects[0].pose.position.y = y_start
    collision_objects[0].pose.position.z = z_start

    ## Define the table ##
    collision_objects_names[1] = "table"
    collision_objects[1].header.frame_id = "panda_link0"
    collision_object_sizes[1] = (1, 1, 0.02)
    collision_objects[1].pose.position.x = 0
    collision_objects[1].pose.position.y = 0
    collision_objects[1].pose.position.z = -0.04

    ## Add collision objects to scene ##

    for (name, pose, size) in zip(
            collision_objects_names, collision_objects, collision_object_sizes
    ):
        planning_scene_interface.add_box(name=name, pose=pose, size=size)

    # planning_scene_interface.add_box(name=collision_objects_names[0], pose=collision_objects[0], size=collision_object_sizes[0])
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
    """
    ## Create robot commander ##
    robot = moveit_commander.RobotCommander(
        robot_description="robot_description", ns="/"
    )
    """
    get_planning_scene = rospy.ServiceProxy('/get_planning_scene', GetPlanningScene)
    set_planning_scene = rospy.ServiceProxy('/apply_planning_scene', ApplyPlanningScene)
    #rospy.logdebug("Robot Groups: %s", robot.get_group_names())

    ## Create scene commanders ##
    # Used to get information about the world and update the robot
    # its understanding of the world.
    move_group = moveit_commander.MoveGroupCommander("panda_arm")
    planning_scene_interface = moveit_commander.PlanningSceneInterface(ns="/")
    ## Specify the planner we want to use ##
    move_group.set_planner_id("TRRT")

    ## Create a `DisplayTrajectory`_ ROS publisher to display the plan in RVIZ ##
    display_trajectory_publisher = rospy.Publisher(
        "/move_group/display_planned_path", DisplayTrajectory, queue_size=20
    )
    eef_link = move_group.get_end_effector_link()

    ## Wait a bit for ROS to initialize the planning_scene_interface ##
    #rospy.sleep(1.0)

    ## Add collision objects ##
    #addCollisionObjects(planning_scene_interface)
    """
    print(move_group.get_current_pose().pose.position.x)
    print(move_group.get_current_pose().pose.position.y)
    print(move_group.get_current_pose().pose.position.z)
    print(move_group.get_current_pose().pose.orientation.x)
    print(move_group.get_current_pose().pose.orientation.y)
    print(move_group.get_current_pose().pose.orientation.z)
    print(move_group.get_current_pose().pose.orientation.w)
    """

    # start_joint_goal = move_group.get_current_joint_values()
    # print(start_joint_goal)

    start_joint_goal = [0.0, -0.785398163397, 0.0, -2.35619449019, 0.0, 1.57079632679, 0.785398163397 + 0.00963116339]
    print("inizio")
    move_group.set_max_velocity_scaling_factor(0.1)
    move_group.set_max_acceleration_scaling_factor(0.03)
    pose = move_group.get_current_pose()
    ## Wait a bit ##
    # go_to_neutral_pose(move_group, current)
    open_gripper()
    print("aperto")
    go_to_starting_pose(move_group)
    print("arrivato")
    ## Pick ##
    print("ora provo a prendere")
    pick(move_group)
    print("Preso")

    ## Wait a bit ##
    rospy.sleep(2.0)
    print("ora me ne vado")
    go_to_neutral_pose2(move_group)
    print("neutrale")
    #rospy.sleep(2.0)
    ## Place ##
    print("ora provo a piazzare")
    place(move_group)
    print("piazzato")
    #rospy.sleep(2.0)
    print("torno indietro")
    go_to_neutral_pose2(move_group)
    #rospy.sleep(2.0)
    #go_to_neutral_pose(move_group)
    go_to_starting_pose(move_group)
