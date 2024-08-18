#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import yaml
import os
import tf_transformations as tf

"""
This node subscribes to the /localization/pose_twist_fusion_filter/pose topic in autoware.universe
and writes the relative poses in the map frame to YAML files in OpenCood format.
The poses are relative to the first pose received in map frame.
"""
class PoseToYamlNode(Node):
    def __init__(self):
        super().__init__('pose_to_yaml_node')
        self.get_logger().info('Pose to YAML Node has started!')

        self.declare_parameter('output_dir', '/tmp')
        self.declare_parameter('topic_name', '/pose')
        self.declare_parameter('first_pose_yaml', 'map_origin.yaml')
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.first_pose_yaml = self.get_parameter('first_pose_yaml').get_parameter_value().string_value

        if self.first_pose_yaml:
            with open(self.first_pose_yaml, 'r') as file:
                first_pose_yaml = yaml.load(file, Loader=yaml.FullLoader)
                first_pose = PoseStamped()
                first_pose.header.stamp.sec = first_pose_yaml['timestamp'].split('.')[0]
                first_pose.header.stamp.nanosec = first_pose_yaml['timestamp'].split('.')[1]
                first_pose.pose.position.x = first_pose_yaml['x']
                first_pose.pose.position.y = first_pose_yaml['y']
                first_pose.pose.position.z = first_pose_yaml['z']
                self.first_pose = self.pose_to_matrix(first_pose.pose)
                self.first_pose_received = True
                self.get_logger().info(f"Loaded first pose: {self.first_pose.tolist()}")

        self.subscription = self.create_subscription(
            PoseStamped,
            self.topic_name,
            self.pose_callback,
            10)

        self.first_pose_received = False
        self.frame_count = 0

        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            self.get_logger().info(f"Created output directory: {self.output_dir}")

    def pose_callback(self, msg):
        self.get_logger().info("Pose received.")
        
        current_pose = self.pose_to_matrix(msg.pose)
        
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        timestamp = f"{sec}.{nanosec:09d}"

        if not self.first_pose_received:
            self.first_pose = current_pose
            self.first_pose_received = True
            with open(os.path.join(self.output_dir, 'map_origin.yaml'), 'w') as file:
                yaml.dump({'x': msg.pose.position.x, 'y': msg.pose.position.y, 'z': msg.pose.position.z, 'timestamp': timestamp}, file)
            self.get_logger().info(f"First pose written to {os.path.join(self.output_dir, 'first_pose.yaml')}")
            
        relative_pose = np.linalg.inv(self.first_pose) @ current_pose

        filename = os.path.join(self.output_dir, f"{self.frame_count:06d}.yaml")
        self.get_logger().info(f"Writing pose to {filename}")

        yaml_dict = {
            'ego_speed': 0,
            'gps': [],
            'lidar_pose': relative_pose,
            'true_ego_pos': relative_pose,
            'vehicles': [],
            'timestamp': timestamp
        }

        with open(filename, 'w') as file:
            yaml.dump(yaml_dict, file)

        self.get_logger().info(f"Pose written to {filename}")

        self.frame_count += 1

    def pose_to_matrix(self, pose):
        matrix = np.eye(4)

        q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
        rotation_matrix = tf.quaternion_matrix(q)[:3, :3]

        matrix[:3, :3] = rotation_matrix
        matrix[0, 3] = pose.position.x
        matrix[1, 3] = pose.position.y
        matrix[2, 3] = pose.position.z

        return matrix
        

def main(args=None):
    rclpy.init(args=args)

    node = PoseToYamlNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
