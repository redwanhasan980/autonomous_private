#!/usr/bin/env python3

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

def odom_callback(data):
    # Extract the linear velocity components
    vx = data.twist.twist.linear.x
    vy = data.twist.twist.linear.y
    
    # Calculate the speed
    speed = (vx**2 + vy**2) ** 0.5
    speed*=3.6
    
    # Publish the speed
    print(speed)

if __name__ == '__main__':
    rospy.init_node('car_speed_node')
    
    # Publisher for car speed

    
    # Subscriber to odometry data
    rospy.Subscriber('/vesc/odom', Odometry, odom_callback)
    
    rospy.spin()
