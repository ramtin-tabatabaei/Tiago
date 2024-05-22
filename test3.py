import rosbag
import csv
import os
import re

bag_path = '/home/pal/rosbags/'
dir_list = os.listdir(bag_path)

# Filtered list for files later than 2024_05_14
filtered_list = [file for file in dir_list if 'record_' in file and file[7:17] >= '2024_05_14']

# Print the filtered list
print(filtered_list)