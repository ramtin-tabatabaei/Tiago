import rosbag
import csv
import os
import re

# Specify the full path to your ROS bag
bag_path = '/home/pal/rosbags/record_2024_05_02_16_07_05.bag'

bag_root = '/home/pal/rosbags/'
dir_list = os.listdir(bag_root)

# Filtered list for files later than 2024_05_14
filtered_list = [file for file in dir_list if 'record_' in file and file[7:17] >= '2024_05_17']


# topic_name = '/ee_state_topic'
# topic_name = '/aruco_pose_TF'
# topic_name = '/joint_states'
# topic_name = '/end_effector_pose'

# List of topics to process
topics_to_process = [
    '/ee_state_topic',
    '/aruco_pose_TF',
    '/joint_states',
    '/end_effector_pose'
]

for dic in filtered_list:
    bag_path = bag_root + dic

    # Regular expression to find the date and time part
    match = re.search(r'record_(\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2})\.bag', bag_path)
    if match:
        date_time = match.group(1)

    # modified_string = topic_name.replace("/", "")


    dir = "rosbag_Results/"+date_time+"/"
    os.makedirs(dir, exist_ok=True)


    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            print("Successfully opened bag.")
            topics_info = bag.get_type_and_topic_info()[1]
            print(f"Topics found in the bag: {list(topics_info.keys())}")

            for topic_name in topics_to_process:
                if topic_name not in topics_info:
                    print(f"Topic {topic_name} not found in the bag.")
                else:
                # for topic_name in topics_to_process:
                    modified_string = topic_name.replace("/", "")
                    print(topic_name)
                    if topic_name == '/ee_state_topic':
                        message_count = 0
                        with open(dir + modified_string+"_"+date_time+'.csv', 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            # Write header row with timestamp and data fields
                            csv_writer.writerow(['timestamp', 'Puzzle Number', 'Object Number', 'Failure type', 'State'])
                            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                                # Check the type name directly
                                if 'Float32MultiArray' in msg._type:
                                #if 'sensor_msgs/JointState' in msg._type:
                                    print(f"Processing Float32MultiArray message at time {t}")
                                    # Convert timestamp to seconds (from nanoseconds) and write to CSV
                                    timestamp_in_seconds = t.to_sec()
                                    csv_writer.writerow([timestamp_in_seconds] + list(msg.data))
                                    message_count += 1
                                else:
                                    print(f"Unexpected message type {msg._type} at time {t}")

                        if message_count == 0:
                            print("No messages processed from the specified topic, although topic exists.")
                        else:
                            print(f"Data has been written, total messages processed: {message_count}")

                    elif topic_name == '/aruco_pose_TF':
                        message_count = 0
                        with open(dir + modified_string + "_" + date_time + '.csv', 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)

                            # Write header row with timestamp and data fields
                            csv_writer.writerow(['timestamp', 'Object Number', 'X', 'Y', 'Z', 'Roll', "Pitch", "Yaw"])
                            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                                # Check the type name directly
                                if 'sensor_msgs/JointState' in msg._type:
                                    print(f"Processing JointState message at time {t}")
                                    # Convert timestamp to seconds (from nanoseconds) and write to CSV
                                    timestamp_in_seconds = t.to_sec()
                                    # Unpack and format joint data into strings for CSV output
                                    joint_names = ';'.join(msg.name)
                                    positions = ';'.join(map(str, msg.position))
                                    velocities = ';'.join(map(str, msg.velocity))
                                    efforts = ';'.join(map(str, msg.effort))
                                    csv_writer.writerow([timestamp_in_seconds, joint_names, positions, velocities, efforts])
                                    message_count += 1
                                else:
                                    print(f"Unexpected message type {msg._type} at time {t}")
                    elif topic_name == '/end_effector_pose':
                        message_count = 0
                        with open(dir + modified_string+"_"+date_time+'.csv', 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            # Write header row with timestamp and data fields
                            csv_writer.writerow(['timestamp', 'Position X', 'Position Y', 'Position Z', 'Orientation X', 'Orientation Y', 'Orientation Z', 'Orientation W'])
                            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                                # Check the type name directly
                                if 'geometry_msgs/PoseStamped' in msg._type:
                                    print(f"Processing PoseStamped message at time {t}")
                                    # Convert timestamp to seconds (from nanoseconds) and write to CSV
                                    timestamp_in_seconds = t.to_sec()
                                    pose = msg.pose
                                    position = pose.position
                                    orientation = pose.orientation
                                    csv_writer.writerow([
                                        timestamp_in_seconds,
                                        position.x, position.y, position.z,
                                        orientation.x, orientation.y, orientation.z, orientation.w
                                    ])
                                    message_count += 1
                                else:
                                    print(f"Unexpected message type {msg._type} at time {t}")


                    elif topic_name == '/joint_states':
                        message_count = 0
                        with open(dir + modified_string + "_" + date_time + '.csv', 'w', newline='') as csvfile:
                            csv_writer = csv.writer(csvfile)
                            # Write header row with timestamp and data fields
                            csv_writer.writerow(['', 'Positions', '' , '', '', '', '', '', '', '', '', '', '', '', '', '', '', 'Velocities', '' , '', '', '', '', '', '', '', '', '', '', '', '', '', ''])
                            csv_writer.writerow(['timestamp', 'arm_1_joint', 'arm_2_joint' , 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_left_finger_joint', 'gripper_right_finger_joint', 'head_1_joint', 'head_2_joint', 'torso_lift_joint', 'wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_rear_left_joint', 'wheel_rear_right_joint', 'arm_1_joint', 'arm_2_joint' , 'arm_3_joint', 'arm_4_joint', 'arm_5_joint', 'arm_6_joint', 'arm_7_joint', 'gripper_left_finger_joint', 'gripper_right_finger_joint', 'head_1_joint', 'head_2_joint', 'torso_lift_joint', 'wheel_front_left_joint', 'wheel_front_right_joint', 'wheel_rear_left_joint', 'wheel_rear_right_joint'])
                            for topic, msg, t in bag.read_messages(topics=[topic_name]):
                                # Check the type name directly
                                if 'sensor_msgs/JointState' in msg._type:
                                    print(f"Processing JointState message at time {t}")
                                    # Convert timestamp to seconds (from nanoseconds) and write to CSV
                                    timestamp_in_seconds = t.to_sec()
                                    # Unpack and format joint data into strings for CSV output
                                    joint_names = ';'.join(msg.name)
                                    positions = ';'.join(map(str, msg.position))
                                    velocities = ';'.join(map(str, msg.velocity))
                                    efforts = ';'.join(map(str, msg.effort))
                                    # csv_writer.writerow([timestamp_in_seconds, joint_names, positions, velocities, efforts])
                                    csv_writer.writerow([timestamp_in_seconds, positions, velocities])

                                    message_count += 1

                                else:
                                    print(f"Unexpected message type {msg._type} at time {t}")

    except Exception as e:
        print(f"An error occurred: {e}")
