#!/usr/bin/env python3
"""
Collision Monitor Node
Memonitor dan mendeteksi collision drone dengan obstacles
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose


class CollisionMonitor(Node):
    """Node untuk monitoring collision"""

    def __init__(self):
        super().__init__('collision_monitor')
        
        # Obstacle positions (dapat di-load dari parameter atau file)
        self.obstacles = [
            {'x': 5.0, 'y': 5.0, 'z': 2.5, 'radius': 1.0},
            {'x': 8.0, 'y': 3.0, 'z': 3.0, 'radius': 0.8},
            {'x': 3.0, 'y': 8.0, 'z': 2.0, 'radius': 1.2},
        ]
        
        # Ground level
        self.ground_z = 0.0
        
        # Collision threshold
        self.collision_threshold = 0.5  # meters
        
        # Publishers
        self.collision_pub = self.create_publisher(
            Bool,
            '/collision/detected',
            10
        )
        
        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry,
            '/model/quadrotor/odometry',
            self.odom_callback,
            10
        )
        
        # State
        self.current_pose = None
        self.collision_detected = False
        
        # Timer untuk monitoring
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Collision Monitor Node started')
        self.get_logger().info(f'Monitoring {len(self.obstacles)} obstacles')

    def odom_callback(self, msg):
        """Callback untuk odometry"""
        self.current_pose = msg.pose.pose

    def check_obstacle_collision(self, pose):
        """Cek collision dengan obstacles"""
        for obstacle in self.obstacles:
            dx = pose.position.x - obstacle['x']
            dy = pose.position.y - obstacle['y']
            dz = pose.position.z - obstacle['z']
            
            distance = (dx**2 + dy**2 + dz**2)**0.5
            
            if distance < (obstacle['radius'] + self.collision_threshold):
                return True
        
        return False

    def check_ground_collision(self, pose):
        """Cek collision dengan ground"""
        return pose.position.z < (self.ground_z + self.collision_threshold)

    def check_collision(self):
        """Cek semua jenis collision"""
        if self.current_pose is None:
            return False
        
        # Check obstacle collision
        if self.check_obstacle_collision(self.current_pose):
            return True
        
        # Check ground collision
        if self.check_ground_collision(self.current_pose):
            return True
        
        return False

    def timer_callback(self):
        """Timer callback untuk monitoring"""
        collision = self.check_collision()
        
        if collision != self.collision_detected:
            self.collision_detected = collision
            if collision:
                self.get_logger().warn('COLLISION DETECTED!')
        
        collision_msg = Bool()
        collision_msg.data = self.collision_detected
        self.collision_pub.publish(collision_msg)


def main(args=None):
    rclpy.init(args=args)
    
    monitor = CollisionMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

