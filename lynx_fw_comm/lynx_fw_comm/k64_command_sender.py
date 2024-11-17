#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
# from gnc_for_ugv.msg import TrackCommand
from lynx_msgs.msg import TrackCommand
import numpy as np
import os
os.environ['MAVLINK20'] = '1'  # Set mavlink2 for odometry message
from pymavlink import mavutil

class MavlinkBridgeSender(Node):
    def __init__(self):
        super().__init__('mavlink_manager')

        # MAVLink connection
        self.connection = mavutil.mavlink_connection('udpout:192.168.1.10:8150')

        # Subscription
        self.subscription = self.create_subscription(
            TrackCommand,
            '/command',
            self.mavlink_send,
            1
        )

    def mavlink_send(self, command_sub: TrackCommand):
        """Callback to send MAVLink messages based on subscribed data."""
        # Extract time and command values
        time_usec = command_sub.header.stamp.sec * 1e6 + command_sub.header.stamp.nanosec / 1e3
        left_speed = int(np.abs(command_sub.left_track))
        right_speed = int(np.abs(command_sub.right_track))
        left_dir = int(np.sign(command_sub.left_track) + 2)
        right_dir = int(np.sign(command_sub.right_track) + 2)

        # Send MAVLink message
        self.connection.mav.rc_channels_send(
            0,  # time_boot_ms
            2,  # port
            left_speed,
            right_speed,
            left_dir,
            right_dir,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
            254  # rssi
        )

def main(args=None):
    rclpy.init(args=args)
    mavlink_manager = MavlinkBridgeSender()
    rclpy.spin(mavlink_manager)
    mavlink_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
