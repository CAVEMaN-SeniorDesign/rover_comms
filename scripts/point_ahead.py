#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion
import math
class setGoalPublisher(Node):
    def __init__(self):
        super().__init__('fake_imu_publisher')
        self.set_goal_pub_ = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.get_odom_sub_ = self.create_subscription(Odometry, 'odometry/filtered', self.odom_callback, 10)
        self.timer = self.create_timer(0.04, self.publish_goal)  # 25Hz
        self.odom_msg = None
        self.goal = None
        
    def odom_callback(self, msg):
        self.odom_msg = msg
        
    def publish_goal(self):
        if self.odom_msg is None:
            return

        # Extract position
        x = self.odom_msg.pose.pose.position.x
        y = self.odom_msg.pose.pose.position.y

        # Extract yaw from quaternion
        q = self.odom_msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        # Compute goal 5 meters forward
        goal_x = x + 5.0 * math.cos(yaw)
        goal_y = y + 5.0 * math.sin(yaw)

        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_x
        goal_msg.pose.position.y = goal_y
        goal_msg.pose.position.z = 0.0
        goal_msg.pose.orientation = q  # Maintain same heading

        self.get_logger().info(f'Publishing goal at ({goal_x:.2f}, {goal_y:.2f})')
        self.set_goal_pub_.publish(goal_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = setGoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
