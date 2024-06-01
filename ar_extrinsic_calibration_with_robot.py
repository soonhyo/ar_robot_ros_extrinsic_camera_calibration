import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_matrix

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

    marker_frame = 'my_tag_0'
    camera_frame = 'realsense_torso_color_optical_frame'
    base_frame = 'base'
    gripper_frame = 'right_gripper'
    marker_to_gripper_offset = [0, 0, -0.02]
    marker_rotation_offset = [0, -np.pi/2, 0]  # Rotation in radians (0, -90 degrees, 0)

    positions = []
    orientations = []

    listener = tf.TransformListener()

    try:
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)  # 10 Hz
        while (rospy.Time.now() - start_time).to_sec() < 5.0:
            try:
                # Get the transform from the camera frame to the marker frame
                marker_to_camera_trans, marker_to_camera_rot = get_transform(listener, camera_frame, marker_frame)

                # Apply the marker to gripper offset and rotation
                marker_to_gripper_matrix = quaternion_matrix(quaternion_from_euler(*marker_rotation_offset))
                marker_to_gripper_matrix[:3, 3] = marker_to_gripper_offset
                marker_to_camera_matrix = quaternion_matrix(marker_to_camera_rot)
                marker_to_camera_matrix[:3, 3] = marker_to_camera_trans

                gripper_to_camera_matrix = np.dot(marker_to_camera_matrix, np.linalg.inv(marker_to_gripper_matrix))

                gripper_to_camera_trans = gripper_to_camera_matrix[:3, 3]
                gripper_to_camera_rot = tf.transformations.quaternion_from_matrix(gripper_to_camera_matrix)

                # Get the transform from the base frame to the gripper frame
                base_to_gripper_trans, base_to_gripper_rot = get_transform(listener, base_frame, gripper_frame)

                # Combine the transforms to get the camera's pose in the base frame
                base_to_gripper_matrix = quaternion_matrix(base_to_gripper_rot)
                base_to_gripper_matrix[:3, 3] = base_to_gripper_trans

                base_to_camera_matrix = np.dot(base_to_gripper_matrix, np.linalg.inv(gripper_to_camera_matrix))

                base_to_camera_trans = base_to_camera_matrix[:3, 3]
                base_to_camera_rot = tf.transformations.quaternion_from_matrix(base_to_camera_matrix)

                positions.append(base_to_camera_trans)
                orientations.append(base_to_camera_rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("TF lookup failed")
            rate.sleep()

        avg_position, avg_orientation = average_poses(positions, orientations)

        rospy.loginfo("Average Position: {}".format(avg_position))
        rospy.loginfo("Average Orientation (quaternion): {}".format(avg_orientation))
    except rospy.ROSInterruptException:
        pass
