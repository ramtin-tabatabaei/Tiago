#!/usr/bin/env python
import rospy
import tf
import tf_conversions
from geometry_msgs.msg import TransformStamped
import tf2_ros

class CoordinateTranslator:

    def __init__(self):
        rospy.init_node('coordinate_translator', anonymous=True)
        self.listener = tf.TransformListener()

        rospy.sleep(1)  # Wait for the listener to get ready

    def publish_static_transformation(self, Parent, Child, x, y, z, roll, pitch, yaw):
        # Create a StaticTransformBroadcaster object
        static_broadcaster = tf2_ros.StaticTransformBroadcaster()

        # Create a TransformStamped message
        static_transform_stamped = TransformStamped()

        # Fill the message with the necessary data
        static_transform_stamped.header.stamp = rospy.Time.now()
        static_transform_stamped.header.frame_id = Parent
        static_transform_stamped.child_frame_id = Child
        static_transform_stamped.transform.translation.x = x
        static_transform_stamped.transform.translation.y = y
        static_transform_stamped.transform.translation.z = z
        
        quaternion = tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw)
        static_transform_stamped.transform.rotation.x = quaternion[0]
        static_transform_stamped.transform.rotation.y = quaternion[1]
        static_transform_stamped.transform.rotation.z = quaternion[2]
        static_transform_stamped.transform.rotation.w = quaternion[3]

        # Send the transformation
        static_broadcaster.sendTransform(static_transform_stamped)
        rospy.loginfo("Static transform published!")

    def get_transformation(self):
        (trans, rot) = self.listener.lookupTransform("/torso_lift_link", "/aruco_marker_frame", rospy.Time(0))
        rot_euler = tf_conversions.transformations.euler_from_quaternion(rot)
        return trans, rot, rot_euler

    def run(self):
        # Define and publish the new frame /xtion_optical_frame_offset, which is offset by 0.1m along the z-axis from /xtion_optical_frame
        offset_x = -0.05  # No offset in x
        offset_y = -0.11  # No offset in y
        offset_z = -0.082  # 0.1 meter offset in z
        offset_roll = 0  # No rotation offset
        offset_pitch = 0  # No rotation offset
        offset_yaw = 0  # No rotation offset

        # Publish the transformation to /xtion_optical_frame_offset
        self.publish_static_transformation('/xtion_optical_frame', '/New_Camera', offset_x, offset_y, offset_z, offset_roll, offset_pitch, offset_yaw)
        rospy.sleep(2)  # Give some time for the transformation to be published


        # Manually entering values here. Replace these with your desired values.
        x = 0.04  # Example values
        y = 0.22
        z = 0.80
        roll = 0.1
        pitch = 0.2
        yaw = 0.3


        self.publish_static_transformation('/New_Camera', '/aruco_marker_frame', 0.2557, 0.1818, 0.8179, roll, pitch, yaw)
        
        rospy.sleep(2)  # Give some time for the transformation to be published

        trans, rot, rot_euler = self.get_transformation()
        print("Point in base frame: ", trans)
        print("Orientation in base frame (roll, pitch, yaw): ", rot_euler)

        rospy.spin()

def main():
    translator = CoordinateTranslator()
    translator.run()

if __name__ == '__main__':
    main()
