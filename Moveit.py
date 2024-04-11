#!/usr/bin/env python

import sys
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler, euler_from_quaternion


def main():
    

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('plan_arm_torso_ik', anonymous=True)

    if len(sys.argv) != 7:
        print("\nUsage:")
        print("\nrosrun tiago_moveit_tutorial plan_arm_torso_ik  x y z  r p y")
        print("\nwhere the list of arguments specify the target pose of /arm_tool_link expressed in /base_footprint\n")
        return

    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    roll = float(sys.argv[4])
    pitch = float(sys.argv[5])
    yaw = float(sys.argv[6])

    quaternion = quaternion_from_euler(roll, pitch, yaw)

    goal_pose = geometry_msgs.msg.PoseStamped()
    goal_pose.header.frame_id = "torso_lift_link"
    goal_pose.pose.position.x = x
    goal_pose.pose.position.y = y
    goal_pose.pose.position.z = z
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]

    # robot = moveit_commander.RobotCommander()
    # scene = moveit_commander.PlanningSceneInterface()
    # group_name = "arm_torso"
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    group.set_planner_id("SBLkConfigDefault")
    #group.set_planner_id("RRTstarkConfigDefault")
    #group.set_planner_id("RRTConnectkConfigDefault")
    

    # group.set_pose_reference_frame("base_footprint")
    # group.set_pose_reference_frame("torso_lift_link")
    group.set_pose_target(goal_pose)

    # # After setting the pose target
    current_pose = group.get_current_pose().pose
    current_orientation = current_pose.orientation
    quaternion = (
        current_orientation.x,
        current_orientation.y,
        current_orientation.z,
        current_orientation.w,
    )
    roll, pitch, yaw = euler_from_quaternion(quaternion)

    print("Current Pose:")
    print(f"x: {current_pose.position.x}, y: {current_pose.position.y}, z: {current_pose.position.z}")
    print(f"Orientation - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

    rospy.loginfo("Planning to move to target pose")
    
    # success, plan, planning_time, error_code = group.plan()
    # if not success:
    #     raise Exception("No plan found")

    # plan = group.plan()
    # if not plan:
    #     raise Exception("No plan found")

    # rospy.loginfo("Plan found, executing...")
    group.go(wait=True)

    rospy.loginfo("Motion completed")

if __name__ == '__main__':
    main()

