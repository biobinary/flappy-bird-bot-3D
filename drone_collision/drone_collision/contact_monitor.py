#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from ros_gz_interfaces.msg import Contacts

class ContactMonitor(Node):

    def __init__(self):

        super().__init__('drone_collision_node')

        self.drone_model_name = 'quadrotor'

        self.collision_pub = self.create_publisher(
            Bool,
            '/drone/collision',
            10
        )

        self.contact_sub = self.create_subscription(
            Contacts,
            '/world/flappy/physics/contacts',
            self.contact_callback,
            10
        )

        self.get_logger().info('Contact Monitor Node (Physics Based) Started')

    def contact_callback(self, msg):
        
        is_collision = False
        
        for contact in msg.contacts:
        
            e1 = contact.entity1.id if hasattr(contact.entity1, 'id') else str(contact.entity1)
            e2 = contact.entity2.id if hasattr(contact.entity2, 'id') else str(contact.entity2)
    
            if self.drone_model_name in str(e1) or self.drone_model_name in str(e2):
                is_collision = True
                self.get_logger().warn(f'Collision detected between: {e1} and {e2}')
                break
        
        # Publish status
        out_msg = Bool()
        out_msg.data = is_collision
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