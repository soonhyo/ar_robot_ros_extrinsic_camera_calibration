#!/usr/bin/env python
import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler, quaternion_matrix

def get_transform(listener, target_frame, source_frame):
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform(target_frame, source_frame, rospy.Time(0))
    return trans, rot

def average_poses(positions, orientations):
    avg_position = np.mean(positions, axis=0).tolist()
    avg_orientation = np.mean(orientations, axis=0).tolist()
    return avg_position, avg_orientation

if __name__ == '__main__':
    rospy.init_node('camera_to_base_tf')

    marker_frames = {
        'd435_front_left_color_optical_frame': 'my_tag_0',
        'd435_front_right_color_optical_frame': 'my_tag_1'
    }
    camera_frames = ['d435_front_left_color_optical_frame', 'd435_front_right_color_optical_frame']
    base_frame = 'base'

    # Define the AR marker offsets and rotations for each camera
    marker_offsets = {
        'd435_front_left_color_optical_frame': {'offset': [-0.017, -0.16, 0], 'rotation': [0, 0, 0]},
        'd435_front_right_color_optical_frame': {'offset': [-0.017, -0.16, 0], 'rotation': [0, 0, 0]}
    }

    listener = tf.TransformListener()

    try:
        for camera_frame in camera_frames:
            positions = []
            orientations = []
            marker_frame = marker_frames[camera_frame]
            start_time = rospy.Time.now()
            rate = rospy.Rate(10)  # 10 Hz
            while (rospy.Time.now() - start_time).to_sec() < 5.0:
                try:
                    # Get the transform from the base frame to the marker frame
                    marker_to_base_trans, marker_to_base_rot = get_transform(listener, base_frame, marker_frame)
                    marker_to_base_matrix = quaternion_matrix(marker_to_base_rot)
                    marker_to_base_matrix[:3, 3] = marker_to_base_trans

                    # Apply the marker to camera offset and rotation
                    marker_to_camera_offset = marker_offsets[camera_frame]['offset']
                    marker_rotation_offset = marker_offsets[camera_frame]['rotation']

                    marker_to_camera_matrix_offset = quaternion_matrix(quaternion_from_euler(*marker_rotation_offset))
                    marker_to_camera_matrix_offset[:3, 3] = marker_to_camera_offset

                    # Compute the transform from marker to the current camera
                    # First, transform the camera offset to the base frame
                    marker_to_camera_in_base_matrix = np.dot(marker_to_base_matrix, marker_to_camera_matrix_offset)
                    base_to_camera_trans = marker_to_camera_in_base_matrix[:3, 3]
                    base_to_camera_rot = tf.transformations.quaternion_from_matrix(marker_to_camera_in_base_matrix)

                    positions.append(base_to_camera_trans)
                    orientations.append(base_to_camera_rot)
                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    rospy.logerr("TF lookup failed")
                rate.sleep()

            avg_position, avg_orientation = average_poses(positions, orientations)

            # Format the output as specified
            position_str = " ".join(["{:.3f}".format(p) for p in avg_position])
            orientation_str = " ".join(["{:.3f}".format(q) for q in avg_orientation])
            output = "{} {} {} {} {}".format(position_str, orientation_str, base_frame, camera_frame, 100)
            rospy.loginfo("myargs: \"{}\"".format(output))
    except rospy.ROSInterruptException:
        pass
