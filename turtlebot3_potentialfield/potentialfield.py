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
pub = None
class Potentialfield(Node):

    def _init_(self):
        qos = QoSProfile(depth=10)

        #publisher and subscriber
        self.create_subscription(LaserScan, 'scan', callback, qos)
        pub = self.create_publisher(Twist, "cmd_vel", 20)

    def make_vector(msg):
        global R
#DEBUG---------------------------------------------------------------------------------------
        print(msg.ranges)

        #distances above R are not important
        ranges = [R if x > R else x for x in msg.ranges]
        #distances below r are to be discarded
        ranges = no_zeros(ranges)
        #lower distance has higher effect
        ranges = [1 - (x/R) for x in ranges]
        return ranges

    def no_zeros(ranges):
        global r
        temp = []
        
        for x in ranges:
            if (x>r):
                temp.append(x)

        return temp

    def callback(msg):
        global A
        global max_vel
        global max_rot
        global pub

#DEBUG---------------------------------------------------------------------------------------
        print(type(self.msg.ranges))

        ranges = make_vector(msg)
        
#DEBUG---------------------------------------------------------------------------------------
        self.get_logger().info('I heard sumtin...')

        #make a vector of each entry in ranges
        for i, e in enumerate(ranges):
            ranges[i] = (e * math.cos(math.radians(i)) , e * math.sin(math.radians(i)))
            
        #sum of all vectors
        vector = (0,0)
        for i, e in enumerate(ranges):
            vector[0] += e[0]
            vector[1] += e[1]

        #flip direction and wrap in matrix
        vector = np.matrix([[-vector[0]],[-vector[1]]])

        #print vector
        print("Vector = %f; %f" % (vector[0], vector[1]))

        #rotate vector by angle offset
        rot = np.matrix([[math.cos(-msg.angle_min) , -math.sin(-msg.angle_min)], [math.sin(-msg.angle_min) , math.cos(-msg.angle_min)]])
        vector = rot * vector

        #add constant attraction from front
        vector[0] += A

        #norm vector to not exceed max velocity
        vector = vector/np.linalg.norm(vector)

        #make direction message
        dir = Twist()
        dir.linear.x = max_vel * vector[0]
        dir.angular.z = max_rot * math.asin(vector[1]/math.hypot(vector[0],vector[1]))

        #print direction
        print("Dir = %f; %f" % (dir.linear.x, dir.angular.z))
            
        pub.publish(dir)


def main():
    rclpy.init()
    
#DEBUG---------------------------------------------------------------------------------------
    print('potentialfield started')        
    
    potential = Potentialfield()
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

