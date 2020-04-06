#! /usr/bin/env python

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

R = 3.5
r = 0.12
A = 1
max_vel = 0.26
max_rot = 1.82

class Potentialfield(Node):

    def __init__(self,node_name):

        super().__init__('potentialfield')

        qos = QoSProfile(depth=10)
        #publisher and subscriber
        #self.sub = self.create_subscription(
        #    LaserScan,
        #    'scan', 
        #    self.callback, 
        #    qos)

        self.pub = self.create_publisher(
            Twist, 
            "cmd_vel", 
            20)
        
        timer_period = 0.5
        self.timer = self.ceate_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = self.i
        msg.linear.y = 42
        msg.angular.z = 21
        self.pub.publish(msg)

        #print direction
        self.get_logger().info("Dir = %f; %f; %f" % (msg.linear.y,msg.linear.x, msg.angular.z))
        self.i += 1
         



def main(args=None):
    rclpy.init(args=args)    
#DEBUG---------------------------------------------------------------------------------------
    print('potentialfield started')        
    
    potential = Potentialfield('potentialfield')
#DEBUG---------------------------------------------------------------------------------------
    print('Potentialfield is created')

    # infinite loop
    rclpy.spin(potential)
#DEBUG---------------------------------------------------------------------------------------
    print('this is after the spin')

    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except rclpy.ROSInterruptException:
        pass

