#!/usr/bin/env python3

import rospy
from urdf_parser_py.urdf import URDF
from kdl_parser_py.urdf import treeFromUrdfModel
from PyKDL import ChainFkSolverPos_recursive, ChainIkSolverPos_LMA, Frame, Vector, Rotation, JntArray
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import actionlib
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from math import pi, inf
import csv

# Subscriber callback for updating current joint positions
def joint_state_callback(msg):
    global current_joint_positions, current_gripper_position, current_head_position
    try:
        for i, name in enumerate(joint_names):
            index = msg.name.index(name)
            current_joint_positions[i] = msg.position[index]
        gripper_index = msg.name.index("gripper_left_finger_joint")
        current_gripper_position[0] = msg.position[gripper_index]*2
        current_gripper_position[1] = msg.position[gripper_index]*2
        # head_1_index = msg.name.index("head_1_joint")
        # head_2_index = msg.name.index("head_2_joint")
        # current_head_position[0] = msg.position[head_1_index]
        # current_head_position[1] = msg.position[head_2_index]
    except Exception as e:
        rospy.logerr(f"Error in joint_state_callback: {e}")

# Your other functions (update_gripper_position, update_head_position, apply_joint_positions, get_current_end_effector_pose) remain the same


def update_gripper_position(increment):
    global current_gripper_position

    # Update the current position based on the increment/decrement value
    # new_position = [pos + increment for pos in current_gripper_position]
    new_position = [increment, increment]
    print(new_position)

    # Ensure the new position is within the allowable range
    # Assuming the gripper range is between 0 (fully closed) and 0.04 (fully open)
    new_position = [max(0.02, min(1, pos)) for pos in new_position]

    # Update the global variable
    current_gripper_position = new_position

    # Create and send the new goal to the action server
    goal = FollowJointTrajectoryGoal()
    trajectory = JointTrajectory()
    trajectory.joint_names = ['gripper_left_finger_joint', 'gripper_right_finger_joint']
    point = JointTrajectoryPoint()
    point.positions = current_gripper_position
    point.time_from_start = rospy.Duration(0.5)
    trajectory.points.append(point)
    goal.trajectory = trajectory
    
    gripper_client.send_goal(goal)
    gripper_client.wait_for_result()


# def update_head_position(pan_increment=0.0, tilt_increment=0.0):
#     global current_head_position
    
#     current_head_position[0] += pan_increment
#     current_head_position[1] += tilt_increment
    
#     current_head_position[0] = max(-1.25, min(1.25, current_head_position[0]))
#     current_head_position[1] = max(-1.25, min(1.25, current_head_position[1]))

#     goal = FollowJointTrajectoryGoal()
#     trajectory = JointTrajectory()
#     trajectory.joint_names = ['head_1_joint', 'head_2_joint']
#     point = JointTrajectoryPoint()
#     point.positions = current_head_position
#     point.time_from_start = rospy.Duration(0.5)
#     trajectory.points.append(point)
#     goal.trajectory = trajectory
    
#     head_client.send_goal(goal)
#     head_client.wait_for_result()


    
    
def apply_joint_positions(joint_position_dict):
    # Create a JointTrajectory message
    traj_msg = JointTrajectory()
    #traj_msg.header.stamp = rospy.Time.now()
    traj_msg.joint_names = joint_names
    
    point = JointTrajectoryPoint()

    all_position = [0] * len(joint_names)

    for i, name in enumerate(joint_names):
        all_position[i] = joint_position_dict[name]
    
    point.positions = all_position
    point.time_from_start = rospy.Duration(5)  # Adjust based on your requirements
    traj_msg.points.append(point)

    # print("message", traj_msg)
    
    # Publish the message
    arm_pub.publish(traj_msg)

# Function to get the current pose of the end-effector
def get_current_end_effector_pose():
    current_pose = Frame()
    fk_solver.JntToCart(current_joint_positions, current_pose)
    return current_pose



def move_to_goal_position(goal_position, goal_orientation):
    global desired_frame, current_joint_positions, ik_solver_pos
    # Set desired frame based on the goal position and orientation
    desired_position = Vector(*goal_position)
    desired_orientation = Rotation.RPY(*goal_orientation)
    desired_frame = Frame(desired_orientation, desired_position)
    
    # Use IK to calculate desired joint positions
    ik_result = ik_solver_pos.CartToJnt(current_joint_positions, desired_frame, desired_joint_positions)
    if ik_result >= 0:  # If IK solution is found
        joint_positions_dict = {joint_names[i]: desired_joint_positions[i] for i in range(number_of_joints)}
        apply_joint_positions(joint_positions_dict)
    else:
        rospy.logerr("Failed to find an IK solution for the desired position.")

def yaw_converter(yaw):
    if yaw > pi/4 and yaw < 3*pi/4:
        print("loop_yaw: ", yaw)
        return yaw
    else:
        if yaw > 3*pi/4:
            yaw -= pi/2
        if yaw < pi/4:
            yaw += pi/2
        return yaw_converter(yaw)
    
def read_csv(filename):
    with open(filename, mode='r') as file:
        csv_reader = csv.reader(file)
        headers = next(csv_reader)
        data = []
        for row in csv_reader:
        # for row in csv_reader:
            data_row = [float(item) for item in row]
            data.append(data_row)
        return data
    
def minimum_rotation_radians(initial_angle, final_angles):
    # Normalize the initial angle to be within the range [-pi, pi)
    print(f"initial_angle : {initial_angle}, final_angles: {final_angles}")
    initial_angle = (initial_angle + pi) % (2 * pi) - pi
    min_rotation = inf
    best_direction = ''
    # Iterate over each final angle
    for final_angle in final_angles:
        # Normalize the final angle
        final_angle = (final_angle + pi) % (2 * pi) - pi
        # Calculate clockwise and counterclockwise rotations
        clockwise = (final_angle - initial_angle) % (2 * pi)
        counterclockwise = (initial_angle - final_angle) % (2 * pi)
        # Find the minimum rotation for this pair
        if clockwise <= counterclockwise:
            rotation = clockwise
            direction = 'clockwise'
        else:
            rotation = counterclockwise
            direction = 'counterclockwise'
        # Update the smallest rotation found
        if rotation < min_rotation:
            min_rotation = rotation
            best_direction = direction
    return min_rotation, best_direction


def run():
    global ik_solver_pos, desired_joint_positions, joint_names, number_of_joints, fk_solver, arm_pub, gripper_client, desired_frame, current_gripper_position, current_joint_positions, current_head_position, head_client
    global data
    # Load the robot model from the parameter server
    robot_urdf = URDF.from_parameter_server()

    # Generate a KDL tree from the URDF model
    success, kdl_tree = treeFromUrdfModel(robot_urdf)
    if not success:
        rospy.logerr("Failed to extract KDL tree from URDF robot model.")
        exit(1)

    # Specify the chain: from base link to end-effector link
    # base_link = "torso_lift_link"
    base_link = "torso_lift_link"
    end_effector_link = "gripper_link"
    # end_effector_link = "gripper_grasping_frame"

    chain = kdl_tree.getChain(base_link, end_effector_link)

    # Initialize the IK solver
    ik_solver_pos = ChainIkSolverPos_LMA(chain)

    # Initialize the joint array with the number of joints
    number_of_joints = chain.getNrOfJoints()
    desired_joint_positions = JntArray(number_of_joints)
    current_joint_positions = JntArray(number_of_joints)

    # Initialize Forward Kinematics solver
    fk_solver = ChainFkSolverPos_recursive(chain)

    # List of joint names for the robot's arm - adjust this list to match your configuration
    joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", 
                   "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]

    current_gripper_position = [0, 0]
    current_head_position = [0, 0]

    # Publisher for controlling the robot's arm
    arm_pub = rospy.Publisher('/arm_controller/command', JointTrajectory, queue_size=10)

    # Action clients for the gripper and head
    gripper_client = actionlib.SimpleActionClient('/parallel_gripper_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    gripper_client.wait_for_server()

    head_client = actionlib.SimpleActionClient('/head_controller/follow_joint_trajectory', FollowJointTrajectoryAction)
    head_client.wait_for_server()

    # Subscribe to the current joint state
    rospy.Subscriber('/joint_states', JointState, joint_state_callback)
    rospy.loginfo("Subscribed to joint states.")

    # Wait to get first values to ensure everything is initialized properly
    rospy.wait_for_message('/joint_states', JointState)
    rospy.sleep(1.0)

    # Get current end-effector position and orientation
    current_end_effector_pose = get_current_end_effector_pose()
    current_position = current_end_effector_pose.p
    current_orientation = current_end_effector_pose.M

    # delta_position = [0, 0, 0]  # Change in position (x, y, z)
    # delta_orientation = [0.0, 0.0, 0]  # Change in orientation (roll, pitch, yaw)

    # # Calculate the goal position and orientation
    # goal_position = [current_position.x() + delta_position[0],
    #                  current_position.y() + delta_position[1],
    #                  current_position.z() + delta_position[2]]
    
    print(f"x: {current_position.x()}, y: {current_position.y()}, z: {current_position.z()}")
    
    # Calculate the new orientation by converting to RPY, adding the deltas, and converting back
    roll, pitch, yaw = current_orientation.GetRPY()
    # goal_orientation = [roll + delta_orientation[0], pitch + delta_orientation[1], yaw + delta_orientation[2]]

    print(f"Orientation - Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")


    # # Initialize desired_frame with the current end-effector pose
    Destination = [[1, 0.735, 0.27], [2, 0.735, 0.27], [3, 0.735, 0.27], [4, 0.595, 0.05484949356562053]]
    # desired_frame = get_current_end_effector_pose()
    object_number = 1
    X, Y= data[object_number-1][1], data[object_number-1][2]
    # X, Y= 0.8, 0

    X_3D = 0.8881*X+0.0234*Y+0.074
    Y_3D = 0.0455*X+0.9401*Y-0.0194
    X_2D = 0.8846*X+0.07
    Y_2D = 0.9319*Y+0.01
    X_Poly = 4.9204*(X) +0.4149*(Y) -2.9245*X**2 -0.1754*X*Y +0.5984*Y**2 -1.2798
    yaw = data[object_number-1][6]
    # Example usage
    final_square = [-3*pi/4, -pi/4, pi/4, 3*pi/4] #1
    final_trapezold = [-pi/4, 3*pi/4] #4
    final_triangle1 = [pi/2]
    final_triangle2 = [-pi/2]

    if object_number == 1:
        rotation, direction = minimum_rotation_radians(yaw, final_square)
    elif object_number == 4:
        rotation, direction = minimum_rotation_radians(yaw, final_trapezold)
    elif object_number == 2 or object_number == 3:
        rotation1, direction1 = minimum_rotation_radians(yaw, final_triangle1)
        rotation2, direction2 = minimum_rotation_radians(yaw, final_triangle2)
        if rotation1 < rotation2:
            rotation = rotation1
            direction = direction1
        else:
            rotation = rotation2
            direction = direction2

    # rotation, direction = minimum_rotation_radians(yaw, final)


    print(f'Minimum rotation is {rotation:.2f} radians {direction}.')

    if rotation > pi/2:
        print("Wrong Orientation")
    #     # if direction == 'counterclockwise':
    #     #     yaw_goal = yaw_converter(yaw) + pi/2
    #     # elif direction == 'clockwise':
    #     #     yaw_goal = yaw_converter(yaw) - pi/2
    yaw_goal = yaw_converter(yaw)
    print("Yaw_goal: ", yaw_goal)


    # yaw_goal =  pi/2
    if direction == 'counterclockwise':
        yaw_des = yaw_goal - rotation
    elif direction == 'clockwise':
        yaw_des = yaw_goal + rotation

    print("Yaw: ", yaw_goal)

    print(yaw_des)

    # # Specify the goal position and orientation for the end-effector
    # goal_position = [0.924*X+0.0298*Y+0.0491, -0.0453*X+1.0084*Y+0.0714, -0.2]  # Adjust as needed
    # goal_position = [X, Y, -0.24]  # Adjust as needed
    # goal_orientation = [0, 0, yaw_des]  # Adjust as needed
    # move_to_goal_position(goal_position, goal_orientation)


    goal_orientation = [0, 0, yaw_goal]  # Adjust as needed
    goal_position = [X_3D, Y_3D, -0.2]  # Adjust as needed
    update_gripper_position(0.09)
    move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(5)
    goal_position = [X_3D, Y_3D, -0.245]  # Adjust as needed
    # update_gripper_position(0.09)
    move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(5)
    # goal_position = [X_3D, Y_3D, -0.245]  # Adjust as needed
    update_gripper_position(0.02)
    # move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(1)
    goal_position = [X_3D, Y_3D, -0.18]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(5)
    goal_position = [Destination[object_number-1][1], Destination[object_number-1][2], -0.18]  # Adjust as needed
    goal_orientation = [0, 0, yaw_des]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(5)
    goal_position = [Destination[object_number-1][1], Destination[object_number-1][2], -0.243]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(5)
    update_gripper_position(0.04)
    move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(1)
    update_gripper_position(0.09)
    # move_to_goal_position(goal_position, goal_orientation)
    rospy.sleep(1)
    goal_position = [Destination[object_number-1][1], Destination[object_number-1][2], -0.18]  # Adjust as needed
    move_to_goal_position(goal_position, goal_orientation)


    # goal_position = [X, Y, -0.2]  # Adjust as needed
    # print(yaw_des)


    goal_orientation = [0, 0, yaw_goal]  # Adjust as needed

    # Move to the specified goal position
    # move_to_goal_position(goal_position, goal_orientation)
    # update_gripper_position(0.09)

    # rospy.sleep(2)

    # goal_position = [0.85, -0.1, -0.2]  # Adjust as needed
    # goal_orientation = [0, 0, 0]  # Adjust as needed

    # # Move to the specified goal position
    # move_to_goal_position(goal_position, goal_orientation)

    save_option = 0
    if save_option == 1:
        # Define the path to your file
        file_path = '/home/pal/tiago_ws/src/woa_tiago/project/Data.txt'

        # Values you want to replace the first two zeros with
        value1 = goal_position[0]  # Change this to your desired value
        value2 = goal_position[1]  # Change this to your desired value

        # Now, read all lines, modify the last one, and write everything back
        with open(file_path, 'r') as file:
            lines = file.readlines()  # Read all lines into a list

        # Modify the last line
        last_line = lines[-1].strip().split(' ')  # Split the last line into a list of its values
        last_line[2] = str(value1)  # Replace the first zero with value1
        last_line[3] = str(value2)  # Replace the second zero with value2
        lines[-1] = ' '.join(last_line) + '\n'  # Join back into a string and assign back to the last line

        # Write everything back to the file
        with open(file_path, 'w') as file:
            file.writelines(lines)  # Write all lines back into the file



if __name__ == "__main__":
    global data
    # Initialize ROS node
    rospy.init_node('tiago_arm_teleop_position')
    file_name = "aruco_data.csv"
    data = read_csv(file_name)
    print(data)

    run()





