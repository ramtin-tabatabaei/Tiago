# import rosbag

# def list_topics(bag_file):
#     with rosbag.Bag(bag_file, 'r') as bag:
#         topics = bag.get_type_and_topic_info()[1].keys()
#         return list(topics)

# # Replace 'path_to_your_bag_file.bag' with the path to your ROS bag file
# bag_file_path = '/home/pal/rosbags/record_2024_04_24_16_59_20.bag'
# topics = list_topics(bag_file_path)
# print("Topics in the bag file:", topics)




import rosbag
import csv
import os
import re

# Specify the full path to your ROS bag
bag_path = '/home/pal/rosbags/record_2024_04_24_16_59_20.bag'
topic_name = '/aruco_pose_TF'

# Regular expression to find the date and time part
match = re.search(r'record_(\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2})\.bag', bag_path)
if match:
    date_time = match.group(1)

modified_string = topic_name.replace("/", "")

try:
    with rosbag.Bag(bag_path, 'r') as bag:
        print("Successfully opened bag.")
        topics_info = bag.get_type_and_topic_info()[1]
        print(f"Topics found in the bag: {list(topics_info.keys())}")

        if topic_name not in topics_info:
            print(f"Topic {topic_name} not found in the bag.")
        else:
            message_count = 0
            with open(modified_string + "_" + date_time + '.csv', 'w', newline='') as csvfile:
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

            if message_count == 0:
                print("No messages processed from the specified topic, although topic exists.")
            else:
                print(f"Data has been written, total messages processed: {message_count}")
except Exception as e:
    print(f"An error occurred: {e}")

