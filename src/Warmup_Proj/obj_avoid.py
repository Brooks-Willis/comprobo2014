#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from math import cos radians sqrt
from numpy import std
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

global valid_ranges

def getch():
    """ Return the next character typed on the keyboard """
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def increment_angle(valid_ranges,index):
    if index > len(valid_ranges):
        loop_index = index - len(valid_ranges)
    elif index < 0:
        loop_index = index + len(valid_ranges)
    else:
        loop_index = index

    return loop_index

def same_cluster(point, old_point):
    angle_dif = radians(point(0)-point(0))
    dist = sqrt(point(1)**2 + old_point(1)**2 - point(1)*old_point(1)*cos(angle_dif))
    if dist < 0.5:
        return True
    else:
        return False


def find_clusters(valid_ranges):
    lidar_data = []
    for key in valid_ranges.keys():
        lidar_point = key, valid_ranges[key]
        lidar_data.append(lidar_point)
    cluster = []
    for point in lidar_data:
        if cluster == []:
            cluster.append(point)
        elif same_cluster(point, cluster(-1)):
            cluster.append(point)
        else:
            clusters.append(cluster)
            cluster = []
    return clusters

def scan_received(msg, pub):
    """ Processes data from the laser scanner, msg is of type sensor_msgs/LaserScan """
    global valid_ranges
    valid_ranges = {}
    for i in range(len(msg.ranges)):
        if msg.ranges[i] > 0 and msg.ranges[i] < 8:
            valid_ranges[i]=(msg.ranges[i])
    #print len(valid_ranges)
    

def main():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
    rospy.init_node('teleop', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        clusters = find_clusters(valid_ranges)
        vectors = find_vectors(clusters)
        pub.publish(velocity_msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
