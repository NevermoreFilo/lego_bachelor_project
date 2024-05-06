#!/usr/bin/env python
import actionlib
import franka_gripper
import rospy
from franka_gripper.msg import MoveAction


legoWidth = 0.0314
legoHeight = 0.019
openOffSet = 0.005
closedOffset = 0.001

def open_gripper():
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
    goal = franka_gripper.msg.GraspGoal(width=legoWidth-closedOffset, speed=0.02, force=1)
    # goal.width = 0.022
    # goal.speed = 1.0

    # Sends the goal to the action server.
    client.send_goal(goal)
    # Waits for the server to finish performing the action.
    client.wait_for_result()
    # Prints out the result of executing the action
    return client.get_result()  # A move result

rospy.init_node('gripper', anonymous=True)
open_gripper()
close_gripper()
