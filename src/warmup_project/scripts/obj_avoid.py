#!/usr/bin/env python
# Software License Agreement (BSD License)

import rospy
from math import cos, sin, radians, sqrt, atan2, pi
from numpy import std
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

valid_ranges = {}

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
    angle_dif = radians(point[0]-old_point[0])
    dist = sqrt(point[1]**2 + old_point[1]**2 - 2*point[1]*old_point[1]*cos(angle_dif))
    #print dist, point, old_point
    if dist < 0.4:
        return True
    else:
        return False

def find_clusters(valid_ranges):
    lidar_data = []
    for key in valid_ranges.keys():
        lidar_point = key, valid_ranges[key]
        lidar_data.append(lidar_point)
    cluster = []
    clusters = []
    for point in lidar_data:
        if not cluster:
            cluster.append(point)
            #print 'New cluster'
        elif same_cluster(point, cluster[-1]):
            cluster.append(point)
            #print 'Adding to cluster'
        else:
            clusters.append(cluster)
            #print 'New cluster'
            cluster = []
            cluster.append(point)
    return clusters

def find_vectors(clusters): #Incoming angles are in degrees
    vectors = []
    mag_scale = 1
    for cluster in clusters:
        push = mag_scale/(sum([point[1] for point in cluster])/len(cluster))**2 #Push from cluster
        x_unit = 0
        y_unit = 0
        for point in cluster:
            x_unit += cos(radians(point[0]))
            y_unit += sin(radians(point[0]))
        ang = atan2(y_unit, x_unit) #Angle of vector from cluster
        x_clust = push * cos(radians(point[0]))
        y_clust = push * sin(radians(point[0]))
        xy_vect = x_clust,y_clust
        if sqrt(x_clust**2 + y_clust**2) < 1: #Only worry about objects nearer than 2.5ms
            vectors.append(xy_vect)
        #print xy_vect

    return vectors

def combine_vectors(vectors): #Outgoing angles in degrees
    x_total = sum([vector[0] for vector in vectors])
    y_total = sum([vector[1] for vector in vectors])

    target_ang = atan2(y_total, x_total)*180/pi + 180
    target_push = sqrt(x_total**2 + y_total**2)
    return target_ang, target_push

def scan_received(msg, pub):
    """ Processes data from the laser scanner, msg is of type sensor_msgs/LaserScan """
    global valid_ranges
    valid_ranges = {}
    for i in range(len(msg.ranges)):
        if msg.ranges[i] > 0 and msg.ranges[i] < 5:
            valid_ranges[i]=(msg.ranges[i])
    #print len(valid_ranges)
    

def main():
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('scan', LaserScan, scan_received, pub)
    rospy.init_node('teleop', anonymous=True)
    r = rospy.Rate(10) # 10hz
    turn = False
    curve = False
    straight = False
    turn_direction = 1
    while not rospy.is_shutdown():
        clusters = find_clusters(valid_ranges)
        vectors = find_vectors(clusters)
        angle_heading, mag_heading = combine_vectors(vectors)
        

        if angle_heading > 180:
            angle_heading = angle_heading - 360

        if angle_heading == 180:
            angle_heading = 0 #If you see nothing go straight ahead

        if -10 < angle_heading < 10 or straight:
            straight = True
            print "Straight", angle_heading, len(vectors)
            velocity_msg = Twist(Vector3(0.2, 0.0, 0.0), Vector3(0.0, 0.0, 0.0)) 
            turn_direction = cmp(angle_heading+.000001,0) #Sign of angle heading, +0.00001 to counter state where angle heading is 0 (No data)
            if angle_heading < -20 or angle_heading > 20:
                straight = False
        elif -80 <= angle_heading <= 80:
            curve = True
            print "curve", angle_heading, len(vectors)
            velocity_msg = Twist(Vector3(0.2*(180.0-abs(angle_heading))/180.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.8*(angle_heading/180.0)))
            turn_direction = cmp(angle_heading+.000001,0) #Sign of angle heading, +0.00001 to counter state where angle heading is 0 (No data)
            if angle_heading < -20 or angle_heading > 20:
                curve = False 
        elif 80 < angle_heading or angle_heading < -80:
            turn = True
            print "turn", angle_heading, len(vectors), turn_direction
            velocity_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, .5*turn_direction))
            if -20 < angle_heading < 20:
                turn = False 
            
        """
                        
                                if not turn:
                                    turn_direction = 
                        
                                if angle_heading < -170 or angle_heading > 170 or turn:
                                    turn = True
                                    velocity_msg = Twist(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.4))
                                    if -45 < angle_heading < 45:
                                        turn = False"""

#        velocity_msg = Twist(Vector3(0.2*(180.0-abs(angle_heading))/180.0, 0.0, 0.0), Vector3(0.0, 0.0, -0.8*(angle_heading/180.0)))
#        print turn, turn_direction, angle_heading, mag_heading, (180.0-abs(angle_heading))/180.0, (angle_heading/180.0)

        pub.publish(velocity_msg)
        r.sleep()
        
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass
