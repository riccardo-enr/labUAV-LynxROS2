#!/usr/bin/env python3

from math import cos, sin, sqrt, pi
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler
import os
from scipy.spatial.transform import Rotation as R
os.environ['MAVLINK20'] = '1'  # Set mavlink2 for odometry message
from pymavlink import mavutil

class MavlinkReceiver(Node):
    def __init__(self):
        super().__init__('mavlink_manager_in')

        # Publishers
        self.pub_enc = self.create_publisher(Odometry, 'encoder', 10)
        self.pub_mag = self.create_publisher(Odometry, 'magnetometer', 10)
        self.pub_imu = self.create_publisher(Imu, 'imu_k64', 10)
        self.pub_imu2 = self.create_publisher(Imu, 'imu_ext', 10)

        # Connection setup
        self.connection_in = mavutil.mavlink_connection('udp:0.0.0.0:8151', input=True)

        # Variables
        self.oldT = 0
        self.pos_l_old = 0
        self.pos_l_new = 0
        self.pos_r_old = 0
        self.pos_r_new = 0
        self.psi_old = 0
        self.x_0 = 0
        self.y_0 = 0
        self.r = 0.06
        self.B = 0.265

        # Timer callback
        self.timer = self.create_timer(0.01, self.mavlink_callback)  # 100Hz

    def mavlink_callback(self):
        current_time = self.get_clock().now().to_msg()

        # IMU K64F
        imu = self.connection_in.recv_match(type='SCALED_IMU', blocking=True)
        imu_ros = Imu()
        imu_ros.header.stamp = current_time
        imu_ros.header.frame_id = "base_link"
        imu_ros.linear_acceleration.x = imu.xacc / 1000 * 9.81
        imu_ros.linear_acceleration.y = imu.yacc / 1000 * 9.81
        imu_ros.linear_acceleration.z = imu.zacc / 1000 * 9.81
        imu_ros.linear_acceleration_covariance[0] = imu.xmag / 10**7
        imu_ros.linear_acceleration_covariance[4] = imu.ymag / 10**7
        imu_ros.linear_acceleration_covariance[8] = imu.xgyro / 10**7
        imu_ros.orientation_covariance[8] = imu.ygyro / 10**5
        self.pub_imu.publish(imu_ros)

        # Magnetometer
        magneto_ros = Odometry()
        magneto_ros.header.stamp = current_time
        magneto_ros.header.frame_id = "odom"
        magneto_ros.pose.pose.orientation.z = imu.zmag / 10000
        magneto_ros.pose.pose.orientation.w = sqrt(1 - (imu.zmag / 10000)**2)
        magneto_ros.pose.pose.orientation.x = 0
        magneto_ros.pose.pose.orientation.y = 0
        self.pub_mag.publish(magneto_ros)

        # Encoders
        encoders = self.connection_in.recv_match(type='WHEEL_DISTANCE', blocking=True)
        encoders_ros = Odometry()
        encoders_ros.header.stamp = current_time
        encoders_ros.header.frame_id = "base_link"
        rot = R.from_quat([0, 0, magneto_ros.pose.pose.orientation.z, magneto_ros.pose.pose.orientation.w])
        [psi, _, _] = rot.as_euler("zyx", degrees=False)

        self.pos_l_new = encoders.distance[0]
        self.pos_r_new = encoders.distance[1]

        if (self.pos_l_new and self.pos_r_new) == 0:
            self.pos_l_old = 0
            self.pos_r_old = 0
            self.psi_old = 0
            self.x_0 = 0
            self.y_0 = 0

        deltaPos_l = self.pos_l_new - self.pos_l_old
        deltaPos_r = self.pos_r_new - self.pos_r_old
        psi_new = self.psi_old + (1 / self.B) * (deltaPos_r - deltaPos_l)

        self.pos_l_old = self.pos_l_new
        self.pos_r_old = self.pos_r_new
        self.psi_old = psi_new

        encoders_ros.pose.pose.position.x = self.x_0 + (deltaPos_r + deltaPos_l) / 2 * cos(psi_new)
        encoders_ros.pose.pose.position.y = self.y_0 + (deltaPos_r + deltaPos_l) / 2 * sin(psi_new)
        self.x_0 = encoders_ros.pose.pose.position.x
        self.y_0 = encoders_ros.pose.pose.position.y

        encoders_ros.pose.covariance[0] = 1e-6
        encoders_ros.pose.covariance[7] = 1e-6
        encoders_ros.pose.covariance[35] = 1e-6

        encoders_ros.twist.twist.linear.x = (encoders.distance[2] + encoders.distance[3]) / 2
        encoders_ros.twist.twist.angular.z = (-encoders.distance[2] + encoders.distance[3]) / self.B
        q = quaternion_from_euler(0, 0, psi_new)
        encoders_ros.pose.pose.orientation.x = q[0]
        encoders_ros.pose.pose.orientation.y = q[1]
        encoders_ros.pose.pose.orientation.z = q[2]
        encoders_ros.pose.pose.orientation.w = q[3]

        encoders_ros.twist.covariance[0] = 1e-5
        encoders_ros.twist.covariance[35] = 1e-8
        self.pub_enc.publish(encoders_ros)

def main(args=None):
    rclpy.init(args=args)
    mavlink_manager = MavlinkReceiver()
    rclpy.spin(mavlink_manager)
    mavlink_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
