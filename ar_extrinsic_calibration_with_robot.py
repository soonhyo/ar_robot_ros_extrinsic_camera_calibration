import rospy
import tf
import numpy as np
from tf.transformations import quaternion_from_euler, quaternion_multiply, quaternion_matrix

def get_marker_pose(marker_frame, reference_frame):
    listener = tf.TransformListener()
    listener.waitForTransform(reference_frame, marker_frame, rospy.Time(), rospy.Duration(4.0))
    (trans, rot) = listener.lookupTransform(reference_frame, marker_frame, rospy.Time(0))
    return trans, rot

def transform_marker_to_robot_base(marker_pose, marker_to_gripper_offset, marker_rotation_offset, base_frame):
    marker_position, marker_orientation = marker_pose

    listener = tf.TransformListener()
    listener.waitForTransform(base_frame, 'right_gripper', rospy.Time(), rospy.Duration(4.0))
    (gripper_position, gripper_orientation) = listener.lookupTransform(base_frame, 'right_gripper', rospy.Time(0))

    # Convert gripper orientation quaternion to rotation matrix
    gripper_rot_matrix = quaternion_matrix(gripper_orientation)

    # Apply the translation offset in the gripper frame
    offset_position = np.dot(gripper_rot_matrix, np.array([marker_to_gripper_offset[0],
                                                           marker_to_gripper_offset[1],
                                                           marker_to_gripper_offset[2],
                                                           1.0]))[:3]

    # Add the offset to the gripper position
    offset_position = [gripper_position[0] + offset_position[0],
                       gripper_position[1] + offset_position[1],
                       gripper_position[2] + offset_position[2]]

    # Create the rotation quaternion for the offset
    offset_quaternion = quaternion_from_euler(marker_rotation_offset[0],
                                              marker_rotation_offset[1],
                                              marker_rotation_offset[2])

    # Combine the marker orientation with the offset rotation
    final_orientation = quaternion_multiply(gripper_orientation, offset_quaternion)
    final_orientation = quaternion_multiply(final_orientation, marker_orientation)

    return offset_position, final_orientation

def average_poses(positions, orientations):
    avg_position = np.mean(positions, axis=0).tolist()
    avg_orientation = np.mean(orientations, axis=0).tolist()
    return avg_position, avg_orientation

if __name__ == '__main__':
    rospy.init_node('ar_marker_to_base_tf')

    marker_frame = 'my_tag_0'
    base_frame = 'base'
    marker_to_gripper_offset = [0, 0, -0.02]
    marker_rotation_offset = [0, -np.pi/2, 0]  # Rotation in radians (0, -90 degrees, 0)

    positions = []
    orientations = []

    try:
        start_time = rospy.Time.now()
        rate = rospy.Rate(30)  # 10 Hz
        while (rospy.Time.now() - start_time).to_sec() < 10.0:
            try:
                marker_pose = get_marker_pose(marker_frame, 'right_gripper')
                base_pose = transform_marker_to_robot_base(marker_pose, marker_to_gripper_offset, marker_rotation_offset, base_frame)
                positions.append(base_pose[0])
                orientations.append(base_pose[1])
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                rospy.logerr("TF lookup failed")
            rate.sleep()

        avg_position, avg_orientation = average_poses(positions, orientations)

        rospy.loginfo("Average Position: {}".format(avg_position))
        rospy.loginfo("Average Orientation (quaternion): {}".format(avg_orientation))
    except rospy.ROSInterruptException:
        pass
