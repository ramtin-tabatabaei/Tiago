import rosbag
import csv
import os

# Specify the full path to your ROS bag
bag_path = '/home/pal/rosbags/record_2024_04_23_16_50_02.bag'
topic_name = '/ee_state_topic'

try:
    with rosbag.Bag(bag_path, 'r') as bag:
        print("Successfully opened bag.")
        topics_info = bag.get_type_and_topic_info()[1]
        print(f"Topics found in the bag: {list(topics_info.keys())}")

        if topic_name not in topics_info:
            print(f"Topic {topic_name} not found in the bag.")
        else:
            message_count = 0
            with open('output.csv', 'w', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                # Write header row with timestamp and data fields
                csv_writer.writerow(['timestamp', 'Puzzle Number', 'Object Number', 'Failure type', 'State'])
                for topic, msg, t in bag.read_messages(topics=[topic_name]):
                    # Check the type name directly
                    if 'Float32MultiArray' in msg._type:
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
                print(f"Data has been written to output.csv, total messages processed: {message_count}")
except Exception as e:
    print(f"An error occurred: {e}")
