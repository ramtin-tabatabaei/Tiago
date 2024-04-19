#!/usr/bin/env python

import rospy
import actionlib
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64
import time
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
import threading
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal


def sort_assemb_start():
    global height_pub, arm_client, gripper_client, head_client
    height_pub = rospy.Publisher('/torso_controller/command', JointTrajectory, queue_size=10)
    current_height = rospy.Subscriber("/joint_states", JointState, joint_states_callback)
    
    arm_client = actionlib.SimpleActionClient('/arm_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    arm_client.wait_for_server()
    rospy.loginfo("arm server connected.")
    
    # gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    # gripper_client.wait_for_server()

    # Initialize the action client for controlling the head's movement
    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    # Rotate the head 30 degrees to the right and tilt it 30 degrees down
    rotate_and_tilt_head(0, -10, 1)  # Note: Assuming negative tilt means downwards

    rospy.loginfo("gripper server connected.")
    rospy.wait_for_message("joint_states", JointState)
    rospy.sleep(1.0)

    up_joint_angles = [0.07, 0.47, -1.53, 1.74, 0.37, -1.37, 0.28]
    move_arm(up_joint_angles, 10)
    


    rospy.loginfo("finish gesture")
    

def joint_states_callback(msg):
        global current_torso_height
        if "torso_lift_joint" in msg.name:
            index = msg.name.index("torso_lift_joint")
            current_torso_height = msg.position[index]

def rotate_and_tilt_head(pan_degrees, tilt_degrees, duration):
    global head_client
    
    # Convert degrees to radians for ROS
    pan_radians = pan_degrees * 3.141592653589793 / 180.0
    tilt_radians = tilt_degrees * 3.141592653589793 / 180.0

    # Define the goal for head movement
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['head_1_joint', 'head_2_joint']  # Assuming these are the correct joint names
    
    point = JointTrajectoryPoint()
    # Assign the new positions for pan (head_1_joint) and tilt (head_2_joint)
    point.positions = [pan_radians, tilt_radians]  
    point.time_from_start = rospy.Duration(duration)  # Adjust the duration as needed for your setup
    trajectory.points.append(point)
    
    goal.trajectory = trajectory

    head_client.send_goal(goal)
    head_client.wait_for_result()


def move_arm(joint_angles, t):
        # Define the goal
        goal = FollowJointTrajectoryGoal()
        trajectory = JointTrajectory()

        # Specify the joint names for arm and torso
        trajectory.joint_names = [
            'arm_1_joint', 'arm_2_joint', 'arm_3_joint', 'arm_4_joint', 
            'arm_5_joint', 'arm_6_joint', 'arm_7_joint'
        ]

        # Define the joint target positions for arm and torso
        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start = rospy.Duration(t)
        trajectory.points.append(point)

        # Set the trajectory in the goal
        goal.trajectory = trajectory

        # Send the goal and wait for the result
        rospy.loginfo("Sending goal for arm and torso movement...")
        arm_client.send_goal(goal)
        if arm_client.wait_for_result(rospy.Duration(t+1)):  # Increase timeout to ensure enough time for execution
            rospy.loginfo("Arm completed successfully.")
        else:
            rospy.loginfo("Arm did not complete before the timeout.")
    


if __name__ == '__main__':
    rospy.init_node('start_position')
    sort_assemb_start()
    rospy.loginfo("finish start_position")