#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import os

os.environ['MAVLINK20'] = '1'  # Set mavlink2 for odometry message


class MavlinkManager(Node):
    def __init__(self):
        super().__init__('mavlink_manager')

        # MAVLink connection
        self.connection = mavutil.mavlink_connection('udpout:192.168.1.11:8150')

        # Subscription to odometry messages
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry/filtered',
            self.mavlink_send,
            1
        )

    def mavlink_send(self, odom_sub: Odometry):
        """Callback to send MAVLink messages based on subscribed odometry data."""
        # Prepare odometry MAVLink message
        time_usec = 0  # Placeholder for timestamp
        frame_id = 0
        child_frame_id = 0

        # Position
        x = odom_sub.pose.pose.position.x
        y = odom_sub.pose.pose.position.y
        z = odom_sub.pose.pose.position.z

        # Quaternion
        q = [
            odom_sub.pose.pose.orientation.w,
            odom_sub.pose.pose.orientation.x,
            odom_sub.pose.pose.orientation.y,
            odom_sub.pose.pose.orientation.z
        ]

        # Linear velocity
        vx = odom_sub.twist.twist.linear.x
        vy = odom_sub.twist.twist.linear.y
        vz = odom_sub.twist.twist.linear.z

        # Angular velocity
        rollspeed = odom_sub.twist.twist.angular.x
        pitchspeed = odom_sub.twist.twist.angular.y
        yawspeed = odom_sub.twist.twist.angular.z

        # Covariances (not used, set to zeros)
        pose_covariance = [0] * 21
        velocity_covariance = [0] * 21

        reset_counter = 0
        estimator_type = 0

        # Send odometry MAVLink message
        self.connection.mav.odometry_send(
            time_usec,
            frame_id,
            child_frame_id,
            x, y, z,
            q,
            vx, vy, vz,
            rollspeed, pitchspeed, yawspeed,
            pose_covariance, velocity_covariance,
            reset_counter, estimator_type
        )


def main(args=None):
    rclpy.init(args=args)
    mavlink_manager = MavlinkManager()
    rclpy.spin(mavlink_manager)
    mavlink_manager.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
