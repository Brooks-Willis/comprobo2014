#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from numpy import std
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

mean_distance = -1.0
min_angle = 0
followdist = 1

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


def scan_received(msg, pub):
    """ Processes data from the laser scanner, msg is of type sensor_msgs/LaserScan """
    global mean_distance
    global min_angle
    global followdist
    valid_ranges = []

    for i in range(len(msg.ranges)): 
        if msg.ranges[i] > 0 and msg.ranges[i] < 8:
            valid_ranges.append(msg.ranges[i])

    min_dist = min(valid_ranges)
    min_angle = valid_ranges.index(min_dist)

    if len(valid_ranges) > 4:
        min_dist_error = [increment_angle(valid_ranges, min_angle-2),
        increment_angle(valid_ranges, min_angle-1),
        increment_angle(valid_ranges, min_angle),
        increment_angle(valid_ranges, min_angle+1),
        increment_angle(valid_ranges, min_angle+2)]
        #print min_dist_error
        if std(min_dist_error): #Is it a wall? (need to add if not a wall, then what?)
            mean_distance = sum(min_dist_error)/float(len(min_dist_error))
        else: 
            mean_distance = followdist
    else:
        mean_distance = followdist

def orient_wall(target_ang):
    global min_angle
    print min_angle, target_ang, mean_distance
    return Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.01*(min_angle - target_ang)))
    
def pointing_at_wall():
    print "checking wall"
    global min_angle
    ang_error = 5 #Degrees
    if 0 <= min_angle < ang_error or (360-ang_error) <= min_angle < 360:
        print "Pointing at wall"
        return True
    else:
        print "Not pointing at wall"
        return False

def wall_follow():
    print "Following Wall"
"""
def approach_wall():
    global followdist
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
    rospy.init_node('teleop', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        #print min_angle, mean_distance
        if pointing_at_wall():
            if not followdist*0.95 < mean_distance < followdist*1.05:
                print "moving forward or backward", mean_distance, followdist
                
                velocity_msg = Twist(Vector3(0.1*(mean_distance - followdist), 0.0, 0.0), Vector3(0.0, 0.0, 0.0))

            else:
                orient_wall(90)
                wall_follow()
                 
        else:
            velocity_msg = orient_wall(0)

        pub.publish(velocity_msg)
        r.sleep()
"""
def approach_wall():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
    rospy.init_node('teleop', anonymous=True)
    r = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        if mean_distance != -1.0:
            velocity_msg = Twist(Vector3(0.2*(mean_distance - 1.0), 0.0, 0.0), Vector3(0.0, 0.0, 0.0))
        pub.publish(velocity_msg)
        r.sleep()

        
if __name__ == '__main__':
    followdist = int(raw_input("Input distance from wall (m)"))
    try:
        approach_wall()
    except rospy.ROSInterruptException: pass
