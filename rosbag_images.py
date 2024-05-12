import os
import cv2
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Initialize CvBridge
bridge = CvBridge()

# Create a folder to save images if it doesn't exist
folder_path = "image_folder"
if not os.path.exists(folder_path):
    os.makedirs(folder_path)

# Specify the full path to your ROS bag
bag_path = '/home/pal/rosbags/record_2024_05_02_16_07_05.bag'

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
                image_filename = os.path.join(folder_path, f"image_{t.to_nsec()}.jpg")
                cv2.imwrite(image_filename, cv_image)
                
            else:
                print(f"Unexpected message type {msg._type} at time {t}")

except Exception as e:
    print(f"An error occurred: {e}")
