#!/usr/bin/env python3
import rospy
import math
from sensor_msgs.msg import LaserScan
from race.msg import drive_param
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import numpy as np
angle_range = 180
car_length = 1.5
vel = 5
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=1)

def find_max_distance_angle(adjusted_distances, start_index=220):
    """
    Find the angle where the distance is maximum in the adjusted distances list,
    within the specified range from start_index to len(adjusted_distances) - start_index.

    Parameters:
        adjusted_distances (list of tuples): List of pairs (angle, distance).
        start_index (int): The starting index for the range within which to find the maximum distance.

    Returns:
        tuple: The angle and the maximum distance within the specified range.
    """
    if not adjusted_distances or len(adjusted_distances) <= 2 * start_index:
        return None

    end_index = len(adjusted_distances) - start_index
    
    # Extract the sublist within the specified range
    sublist = adjusted_distances[start_index:end_index]

    # Find the tuple with the maximum distance in the sublist
    max_distance_pair = max(sublist, key=lambda x: x[1])
    
    # Return the angle and the maximum distance of that tuple
    return max_distance_pair

def adjust_for_disparities(distances, angle_increment, angle_range=0.12):
    """
    Adjust distances to handle disparities and update surrounding distances.

    Parameters:
        distances (list of tuples): List of pairs (angle, distance).
        angle_increment (float): The angular distance between measurements.
        angle_range (float): The angular range around the disparity to adjust.

    Returns:
        list: Adjusted list of distances.
    """
    adjusted_distances = distances.copy()

    num_angles_to_adjust = int(angle_range / angle_increment)

    for i in range(len(distances) - 1):
        current_angle, current_dist = distances[i]
        next_angle, next_dist = distances[i + 1]

        if abs(current_dist - next_dist) > 1:
            min_dist = min(current_dist, next_dist)

            # Adjust distances around the disparity
            for j in range(max(0, i - num_angles_to_adjust), min(len(distances), i + num_angles_to_adjust + 1)):
                angle, dist = distances[j]
                if dist > min_dist:
                    adjusted_distances[j] = (angle, min_dist)
            i=i+num_angles_to_adjust+1

    return adjusted_distances




def calculate_angles(angle_min, angle_max, angle_increment):
    """
    Calculate the angles for each LIDAR scan data point.

    Parameters:
        angle_min (float): The minimum angle of the LIDAR scan.
        angle_max (float): The maximum angle of the LIDAR scan.
        angle_increment (float): The angular distance between measurements.

    Returns:
        list: A list of angles corresponding to each LIDAR scan data point.
    """
    angles = []
    current_angle = angle_min
    while current_angle <= angle_max:
        angles.append(current_angle)
        current_angle += (angle_increment)
    return angles

def getRange(data):
    """
    0.24999951781635246228
    Get the distances for each angle from the LIDAR scan data.

    Parameters:
        data (LaserScan): The LIDAR scan data.

    Returns:
        list: List of pairs (angle, distance).
    """
    angle_min = data.angle_min
    angle_max = data.angle_max
    angle_increment = data.angle_increment
     # angle increase 0.24999951781635246228
    ranges = data.ranges
    
    angles = calculate_angles(angle_min, angle_max, angle_increment)
    distances = []
    for i, angle in enumerate(angles):
        dist = ranges[i]
        if math.isinf(dist):
            dist = 10.0
        elif math.isnan(dist):
            dist = 10.0
        distances.append((angle, dist))
    return distances

def check_distances_for_sides(adjusted_distances, threshold):
    """
    Check if any distance element in the first half of the list is less than the threshold,
    and if any distance element in the last half of the list is less than the threshold.

    Parameters:
        adjusted_distances (list of tuples): List of pairs (angle, distance).
        threshold (float): The distance threshold to check against.

    Returns:
        tuple: Two boolean variables indicating if the condition is met for the right and left halves.
    """
    n = len(adjusted_distances)
    half_n = n // 2

    # Check the first half for distances less than the threshold
    right = any(distance < threshold for _, distance in adjusted_distances[:half_n])

    # Check the second half for distances less than the threshold
    left = any(distance < threshold for _, distance in adjusted_distances[half_n:])

    return left, right

def callback(data):
    global vel
  
    distances = getRange(data)
    post_dis= adjust_for_disparities(distances,data.angle_increment,0.40)
    # for angle, dist in distances:
    #     print(f"Angle: {angle}, Distance: {dist}")
    # for angle, dist in post_dis:
    #     print(f"Angle_post: {angle}, Distance_post: {dist}")
    max_angel=find_max_distance_angle(post_dis)
    print(f"Angle_post: {max_angel[0]}, Distance_post: {max_angel[1]}")
    left,right=check_distances_for_sides(post_dis,0.30)
    # print(f"left: {left}, right: {right}")
    msg = drive_param()
    if max_angel[1]>8:
        msg.velocity = 8
    else :
        msg.velocity = vel
    # if left == True:
    #     msg.angle=-1.2
    # elif right == True:
    #     msg.angle=1.2
    # else :
    msg.angle=max_angel[0]
    pub.publish(msg)
    # pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('disparity_extender', anonymous=True)
    rospy.Subscriber("scan", LaserScan, callback)

    rospy.spin()

