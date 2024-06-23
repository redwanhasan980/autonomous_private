#!/usr/bin/env python3

import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import pid_input

angle_range = 180
car_length = 1.5
vel = 1
error = 0.0
alpha = 0.0

pub = rospy.Publisher('error', pid_input, queue_size=10)

def getRange(data, angle):
    if angle > 179.9:
        angle = 179.9
    index = len(data.ranges) * angle / angle_range
    dist = data.ranges[int(index)]
    if math.isinf(dist) or math.isnan(dist):
        return 4.0
    return data.ranges[int(index)]

def followRight(data, desired_trajectory):
    global alpha

    a = getRange(data, 60)
    b = getRange(data, 0)
    swing = math.radians(60)
    alpha = math.atan((a * math.cos(swing) - b) / (a * math.sin(swing)))
    print("a", "b", a, b)
    print("Alpha right", math.degrees(alpha))
    curr_dist = b * math.cos(alpha)

    future_dist = curr_dist + car_length * math.sin(alpha)

    print("Right: ", future_dist)
    error = desired_trajectory - future_dist
    print("Error: ", error)
    return error, curr_dist

def callback(data):
    global error
    global alpha
    print(" ")

    error_right, curr_dist_right = followRight(data, 0.05)
    error_left, curr_dist_left = followRight(data, 0.05)
    if curr_dist_right >= curr_dist_left:
        error = error_left
        print('Following Left')
        print('Error', error)
    else:
        error = error_right
        print('Following Right')
        print('Error', error)

    print('Is error same?', error)
    msg = pid_input()
    msg.pid_error = error
    msg.pid_vel = vel
    pub.publish(msg)

if __name__ == '__main__':
    print("Laser node started")
    rospy.init_node('dist_finder', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)
    rospy.spin()

