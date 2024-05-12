import cv2
import os
def make_video(image_folder, video_name, fps):
    images = [img for img in os.listdir(image_folder) if img.endswith(".jpg")]
    images.sort()  # Sort the images by name (ensure the order is correct)
    # Extract an example image to get dimensions
    frame = cv2.imread(os.path.join(image_folder, images[0]))
    height, width, layers = frame.shape
    # Define the codec and create VideoWriter object
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')  # or (*'XVID')
    out = cv2.VideoWriter(video_name, fourcc, fps, (width, height))
    for image in images:
        frame = cv2.imread(os.path.join(image_folder, image))
        out.write(frame)  # Write out frame to video
    out.release()  # Release the video writer
# Usage
image_folder = '/home/pal/tiago_ws/src/woa_tiago/project/image_folder'  # Path to the folder containing images
video_name = image_folder+".mp4"
fps = 4  # Frames per second
make_video(image_folder, video_name, fps)