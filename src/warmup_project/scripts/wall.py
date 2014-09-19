#!/usr/bin/env python

import roslib; roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from numpy import std
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan

mean_distance = -1.0
min_angle = 0

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

# define state wall follow
class Wall_Follow(smach.State):
	def __init__(self):
		smach.State.__init__(self, outcomes=['obstacle_found','manual_override'])
		self.counter = 0

	def execute(self, userdata):
        rospy.loginfo('Executing state Wall_Follow')
        velocity_msg = Twist(Vector3(0.1*(mean_distance - followdist), 0.0, 0.0), Vector3(0.0, 0.0, 0.1*(min_angle - 90)))
 
 
# define state Approch_Wall
class Approach_Wall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['follow', 'orient'])

    def execute(self, userdata):
        rospy.loginfo('Executing state BAR')
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
	                return 'orient'
	                
	                 
	        else:
	            velocity_msg = orient_wall(0)

	        pub.publish(velocity_msg)
	        r.sleep()
 
# main
def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome4', 'outcome5'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('Wall_Follow', Wall_Follow(), 
                               transitions={'outcome1':'Approach_Wall', 
                                            'outcome2':'outcome4'})
        smach.StateMachine.add('Approach_Wall', Approach_Wall(), 
                               transitions={'outcome2':'Wall_Follow'})
 
    # Execute SMACH plan
    outcome = sm.execute()
 
if __name__ == '__main__':
    followdist = int(raw_input("Input distance from wall (m)"))
    try:
        approach_wall()
    except rospy.ROSInterruptException: pass