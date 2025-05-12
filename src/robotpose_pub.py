#!/usr/bin/env python3

# General imports
import math
import cv2
import numpy as np
import pyapriltags as apriltags
from pyk4a import PyK4A, Config, ColorResolution, DepthMode
from scipy.spatial.transform import Rotation as R

# ROS 2 related imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import atexit
import time

def calculate_relative_pose(reference_pose_R, reference_pose_T, tag_pose_R, tag_pose_T):
    R_inv_ref = np.transpose(reference_pose_R)
    T_inv_ref = -np.dot(R_inv_ref, reference_pose_T)
    relative_pose_R = np.dot(R_inv_ref, tag_pose_R)
    relative_pose_T = tag_pose_T - reference_pose_T
    relative_pose_T[1] = -relative_pose_T[1]
    relative_pose_T[2] = -relative_pose_T[2]
    return relative_pose_T, relative_pose_R

def rotate_point_cloud(points, yaw_angle, pitch_angle, roll_angle):
    yaw_rad = np.radians(yaw_angle)
    pitch_rad = np.radians(pitch_angle)
    roll_rad = np.radians(roll_angle)
    R_yaw = np.array([[np.cos(yaw_rad), -np.sin(yaw_rad), 0],
                      [np.sin(yaw_rad), np.cos(yaw_rad), 0],
                      [0, 0, 1]])
    R_pitch = np.array([[np.cos(pitch_rad), 0, np.sin(pitch_rad)],
                        [0, 1, 0],
                        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]])
    R_roll = np.array([[1, 0, 0],
                       [0, np.cos(roll_rad), -np.sin(roll_rad)],
                       [0, np.sin(roll_rad), np.cos(roll_rad)]])
    R_combined = np.dot(R_yaw, np.dot(R_pitch, R_roll))
    rotated_points = np.dot(points, R_combined.T)
    rotated_points[:, 0] -= 1.16
    rotated_points[:, 1] += 1.15
    rotated_points[:, 2] += 2.15
    return rotated_points

def normalize_coordinates(transformed_coordinates):
    points = np.array([coord.flatten() for coord in transformed_coordinates[1:]])
    min_x = np.min(points[:, 0])
    max_x = np.max(points[:, 0])
    min_y = -1*np.max(points[:, 1])
    max_y = -1*np.min(points[:, 1])
    normalized_coordinates = []
    for coord in transformed_coordinates:
        if coord.size == 1:
            normalized_coordinates.append(coord)
        else:
            x_normalized = (coord[0][0] - min_x) / (max_x - min_x)
            y_normalized = -1*(coord[1][0] - min_y) / (max_y - min_y)
            normalized_coordinates.append(np.array([[x_normalized], [y_normalized]]))
    return normalized_coordinates

def rotation_matrix_to_quaternion(R_matrix):
    r = R.from_matrix(R_matrix)
    quat = r.as_quat()
    quat[1], quat[2] = -quat[1], -quat[2]
    return quat

def broadcast_all_tag_transforms(tags, reference_R, reference_T, tf_broadcaster, node):
    for tag in tags:
        if tag.pose_R is None or tag.pose_t is None or tag.tag_id == 1:
            continue
        relative_pose_T, relative_pose_R = calculate_relative_pose(reference_R, reference_T, tag.pose_R, tag.pose_t)
        quat = rotation_matrix_to_quaternion(relative_pose_R)
        transform = TransformStamped()
        transform.header.stamp = node.get_clock().now().to_msg()
        transform.header.frame_id = "world"
        transform.child_frame_id = f"tag_{tag.tag_id}"
        transform.transform.translation.x = relative_pose_T[0][0]
        transform.transform.translation.y = relative_pose_T[1][0]
        transform.transform.translation.z = relative_pose_T[2][0]
        transform.transform.rotation.x = quat[0]
        transform.transform.rotation.y = quat[1]
        transform.transform.rotation.z = quat[2]
        transform.transform.rotation.w = quat[3]
        tf_broadcaster.sendTransform(transform)

def crop_point_cloud(points, x_min, x_max, y_min, y_max, z_min, z_max):
    return np.array([p for p in points if x_min <= p[0] <= x_max and y_min <= p[1] <= y_max and z_min <= p[2] <= z_max])

if __name__ == '__main__':
    rclpy.init()
    node = Node('robot_pose_publisher')
    tf_broadcaster = TransformBroadcaster(node)
    pose_pub = node.create_publisher(PoseStamped, 'robot_pose', 10)
    pcd_pub = node.create_publisher(PointCloud2, 'environment_pcd', 10)
    device = PyK4A(Config(color_resolution=ColorResolution.RES_1080P, depth_mode=DepthMode.NFOV_2X2BINNED, synchronized_images_only=True))
    device.start()

    def shutdown_hook():
        node.get_logger().info("Shutting down the Kinect camera...")
    atexit.register(shutdown_hook)

    yaw_angle = node.declare_parameter("yaw_angle", 180.0).value
    pitch_angle = node.declare_parameter("pitch_angle", 180.0).value
    roll_angle = node.declare_parameter("roll_angle", -5.5).value

    at_detector = apriltags.Detector(families='tag16h5', nthreads=4, quad_decimate=1.0, quad_sigma=0.5, refine_edges=1, decode_sharpening=0.25)
    origin_tag_id = 1
    robot_tag_id = 0
    tag_detect = True
    check_robot = True


    while rclpy.ok():
        # rate = node.create_rate(10)
        pose_stamped = PoseStamped()
        capture = device.get_capture()
        if not capture or not hasattr(capture, 'depth_point_cloud') or capture.depth_point_cloud is None:
            node.get_logger().warn("Invalid point cloud data! Skipping frame.")
            continue

        color_image = capture.color
        points = capture.depth_point_cloud.reshape((-1, 3))
        valid_mask = np.any(points != 0, axis=1)
        valid_points = points[valid_mask] / 1000.0

        rotated_points = rotate_point_cloud(valid_points, yaw_angle, pitch_angle, roll_angle)
        cropped_points = crop_point_cloud(rotated_points, -2.2, -0.05, 0, 2.25, -0.5, 0.5)

        header = Header()
        header.stamp = node.get_clock().now().to_msg()
        header.frame_id = "world"
        
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]

        point_cloud_msg = pc2.create_cloud(header, fields, cropped_points)
        pcd_pub.publish(point_cloud_msg)
        # node.get_logger().info(f"Published rotated point cloud with {len(rotated_points)} points")

        if tag_detect:
            gray_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2GRAY)
            tags = at_detector.detect(gray_image, estimate_tag_pose=True, camera_params=[904.1013, 903.9727, 970.8933, 561.6608], tag_size=0.12)
            tags = [tag for tag in tags if tag.decision_margin > 30]
            # transformed_coordinates = []
            # reference_R, reference_T, robot_pose_T = None, None, None
            transformed_coordinates = []
            reference_R, reference_T, robot_pose_T, robot_pose_R = None, None, None, None

            for tag in tags:
                if tag.tag_id == origin_tag_id:
                    reference_R = tag.pose_R
                    reference_T = tag.pose_t
                    break

            if reference_R is None or reference_T is None:
                print("Reference tag not detected. Skipping frame.")
                continue

            for tag in tags:
                relative_pose_T, relative_pose_R = calculate_relative_pose(reference_R, reference_T, tag.pose_R, tag.pose_t)
                transformed_coordinates.append(relative_pose_T)
                if tag.tag_id == robot_tag_id and check_robot:
                    robot_pose_T = relative_pose_T
                    robot_pose_R = relative_pose_R

            if robot_pose_T is None and check_robot:
                print("Robot's tag not detected. Skipping frame.")
                continue

            # print("Functioning")

            normalized_coordinates = normalize_coordinates(transformed_coordinates)
            if check_robot:
                robot_index = [i for i, tag in enumerate(tags) if tag.tag_id == robot_tag_id][0]
                robot_normalized_coordinates = normalized_coordinates[robot_index]
                quat = rotation_matrix_to_quaternion(robot_pose_R)
                x, y, z = float(robot_pose_T[0][0]), float(robot_pose_T[1][0]), float(robot_pose_T[2][0])
                print(f"Robot's pose: x={x}, y={y}, z={z}, quat={quat}")
                
                # Publish pose
                pose_stamped.pose.position = Point(x=x, y=y, z=z)
                pose_stamped.pose.orientation = Quaternion(x=quat[0], y=quat[1], z=quat[2], w=quat[3])
                pose_pub.publish(pose_stamped)
                # print("published")
                # Broadcast the TF
                transform = TransformStamped()
                transform.header.stamp = node.get_clock().now().to_msg()
                transform.header.frame_id = "world"
                transform.child_frame_id = "base_link"
                transform.transform.translation.x = x
                transform.transform.translation.y = y
                transform.transform.translation.z = z
                transform.transform.rotation.x = quat[0]
                transform.transform.rotation.y = quat[1]
                transform.transform.rotation.z = quat[2]
                transform.transform.rotation.w = quat[3]
                tf_broadcaster.sendTransform(transform)

            if node.get_parameter_or('broadcast_all_tags', False):
                broadcast_all_tag_transforms(tags, reference_R, reference_T, tf_broadcaster, node)
        
        time.sleep(0.1)
 


