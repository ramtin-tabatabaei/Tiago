import os
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import re

# Initialize CvBridge
bridge = CvBridge()

# Create a folder to save images if it doesn't exist
folder_path = "image_folder"
if not os.path.exists(folder_path):
    os.makedirs(folder_path)

# Specify the full path to your ROS bag
bag_path = '/home/pal/rosbags/record_2024_05_02_16_07_05.bag'



bag_root = '/home/pal/rosbags/'
dir_list = os.listdir(bag_root)

# Filtered list for files later than 2024_05_14
filtered_list = [file for file in dir_list if 'record_' in file and file[7:17] >= '2024_05_17']


for dic in filtered_list:
    bag_path = bag_root + dic

    # Regular expression to find the date and time part
    match = re.search(r'record_(\d{4}_\d{2}_\d{2}_\d{2}_\d{2}_\d{2})\.bag', bag_path)
    if match:
        date_time = match.group(1)

    # modified_string = topic_name.replace("/", "")


    dir = "rosbag_Results/"+date_time+"/"+"frames/"
    os.makedirs(dir, exist_ok=True)


    try:
        with rosbag.Bag(bag_path, 'r') as bag:
            print("Successfully opened bag.")
            
            # Iterate over messages in the bag
            for topic, msg, t in bag.read_messages(topics=['/xtion/rgb/image_raw_throttled']):
                if 'sensor_msgs/Image' in msg._type:
                    print(f"Processing Image message at time {t}")
                    # Convert ROS Image message to OpenCV image
                    cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                    
                    # Save the image to a file
                    image_filename = os.path.join(dir, f"image_{t.to_nsec()}.jpg")
                    cv2.imwrite(image_filename, cv_image)
                    
                else:
                    print(f"Unexpected message type {msg._type} at time {t}")

    except Exception as e:
        print(f"An error occurred: {e}")
