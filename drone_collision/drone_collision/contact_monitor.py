#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts

class ContactMonitor(Node):

    def __init__(self):
        super().__init__('contact_monitor')

        self.drone_model_name = 'quadrotor'
        self.collision_active = False

        self.collision_pub = self.create_publisher(
            Bool,
            '/drone/collision',
            10
        )

        # Subscribe ke topik kontak physics dari Gazebo
        self.contact_sub = self.create_subscription(
            Contacts,
            '/drone/contacts',
            self.contact_callback,
            10
        )
        
        # CRITICAL FIX: Listen to reset trigger to clear collision state
        self.reset_sub = self.create_subscription(
            Bool,
            '/ga/reset_trigger',
            self.reset_callback,
            10
        )

        self.get_logger().info('Contact Monitor Listening on: /world/flappy/physics/contacts')

    def reset_callback(self, msg):
        """Reset collision state when new episode starts"""
        if msg.data:
            self.collision_active = False
            self.get_logger().info('Collision state reset for new episode')

    def contact_callback(self, msg):

        if not msg.contacts:
            return

        is_collision = False
        
        for contact in msg.contacts:
            # e1 = contact.entity1.name
            # e2 = contact.entity2.name
            is_collision = True
            break
        
        if is_collision:
            if not self.collision_active:
                # other_entity = e2 if self.drone_model_name in str(e1) else e1
                self.get_logger().warn(f'!!! COLLISION DETECTED !!!')
                self.collision_active = True
            
            out_msg = Bool()
            out_msg.data = True
            self.collision_pub.publish(out_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ContactMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()