#!/usr/bin/env python3
"""
Drone Controller Node
Mengontrol pergerakan drone berdasarkan command dari GA atau manual input
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Bool


class DroneController(Node):
    """Node untuk mengontrol drone"""

    def __init__(self):
        super().__init__('drone_controller')
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/model/quadrotor/cmd_vel',
            10
        )
        
        self.status_pub = self.create_publisher(
            Bool,
            '/drone/status',
            10
        )
        
        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/drone/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/quadrotor/odometry',
            self.odom_callback,
            10
        )
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/model/quadrotor/imu',
            self.imu_callback,
            10
        )
        
        # State variables
        self.current_pose = None
        self.current_odom = None
        self.current_imu = None
        
        # Timer untuk publish status
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Drone Controller Node started')

    def cmd_vel_callback(self, msg):
        """Callback untuk menerima command velocity"""
        # Forward command ke Ignition
        self.cmd_vel_pub.publish(msg)
        self.get_logger().debug(f'Published cmd_vel: linear={msg.linear}, angular={msg.angular}')

    def odom_callback(self, msg):
        """Callback untuk menerima odometry data"""
        self.current_odom = msg
        self.current_pose = msg.pose.pose

    def imu_callback(self, msg):
        """Callback untuk menerima IMU data"""
        self.current_imu = msg

    def timer_callback(self):
        """Timer callback untuk publish status"""
        status_msg = Bool()
        status_msg.data = self.current_pose is not None
        self.status_pub.publish(status_msg)

    def get_current_pose(self):
        """Mengembalikan pose saat ini"""
        return self.current_pose

    def get_current_odom(self):
        """Mengembalikan odometry saat ini"""
        return self.current_odom


def main(args=None):
    rclpy.init(args=args)
    
    controller = DroneController()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

