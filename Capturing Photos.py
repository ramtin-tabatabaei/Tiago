#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:
    def __init__(self):
        # Initialize the node named image_converter
        rospy.init_node('Camera_calibration', anonymous=True)
        
        # Create a CvBridge to convert ROS images to OpenCV format
        self.bridge = CvBridge()

        # Subscribe to the camera image topic
        self.image_sub = rospy.Subscriber("/xtion/rgb/image_raw", Image, self.callback)
        self.count = 153

    def callback(self, data):
        try:
            # Convert the ROS image to an OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # Display the OpenCV image
        cv2.imshow("Image window", cv_image)
        # cv2.waitKey(3)

        # Check for key presses
        key = cv2.waitKey(3) & 0xFF
        if key == ord('s'):
            cv2.imwrite(f"Photos/frame{self.count}.jpg", cv_image)
            print(f"Saved frame{self.count}.jpg")
            self.count += 1

def main():
    ic = ImageConverter()
    try:
        # Spin until ctrl + c
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

