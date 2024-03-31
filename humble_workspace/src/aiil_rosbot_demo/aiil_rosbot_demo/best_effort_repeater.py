#!/usr/bin/env python  

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String


'''
This Node repeats a sensor_data (best_effort) QoS topic to
    a reliable QoS topic
'''
class BestEffortRepeater(Node):
    def __init__(self):
        super().__init__('best_effort_repeater')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('sub_topic_name', '/be_topic'),
                ('repeat_topic_name', '/repeat_topic'),
            ]
        )
        self.sub_topic_name = self.get_parameter('sub_topic_name').value
        self.repeat_topic_name = self.get_parameter('repeat_topic_name').value
        
        # Create subscriber
        self.sub_be = self.create_subscription(
                            #String,
                            Image,
                            self.sub_topic_name,
                            self.be_listener,
                            rclpy.qos.qos_profile_sensor_data
                      )

        # Repeater
        self.pub_reliable = self.create_publisher(
                              #String, 
                              Image,
                              self.repeat_topic_name,
                              1
                          )

    def be_listener(self, msg):
        # self.get_logger().info('Received: "%s"' % msg.data)
        
        # Repeat the data
        self.pub_reliable.publish(msg)
        # self.get_logger().info('Published: "%s"' % msg.data)


def main(args=None):
    rclpy.init()
    node = BestEffortRepeater()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()
    exit(0)