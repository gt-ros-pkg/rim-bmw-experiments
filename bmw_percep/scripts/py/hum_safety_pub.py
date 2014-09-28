#!/usr/bin/env python

# Publishes whether it is safe for the robot to move

import roslib
roslib.load_manifest('bmw_percep')

import rospy
import tf
import matplotlib.pyplot

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool as BoolMsg
from math import sqrt
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker

class HumanSafetyPub:
    def __init__(self):

        #subscribing to -- 
        self.pers_pos_sub = rospy.Subscriber('/human/estimated/pose', 
                                             PoseArray, self.pers_cb, 
                                             queue_size=1)
        
        self.robo_listener = tf.TransformListener()

        #publishing to -- 
        # topic is latched
        self.hum_signal_pub = rospy.Publisher('/human/safety/stop', 
                                              BoolMsg, queue_size=1, latch=True)
        self.hum_signal_viz = rospy.Publisher('/human/safety/visual', 
                                              MarkerArray, queue_size=1)
        
        self.robo_frame = "base_link"
        self.slope = 5.

        #initialize to don't stop human
        self.stop_human = False

    #callback receives person position and velocity
    def pers_cb(self, msg):

        self.frame = msg.header.frame_id
        self.hum_time = msg.header.stamp
        self.hum_pos = np.array([msg.poses[0].position.x, msg.poses[0].position.y])
        self.hum_vel = np.array([msg.poses[1].position.x, msg.poses[1].position.y])
        self.cur_time = rospy.Time(0)

        #Get robot transform
        try:
            self.robo_listener.waitForTransform(self.frame, self.robo_frame, 
                                                self.cur_time,
                                                rospy.Duration(1./30.))
            (trans, rot) = self.robo_listener.lookupTransform(self.frame, 
                                                              self.robo_frame, 
                                                              self.cur_time)
            self.robo_pos = np.array([trans[0], trans[1]])
            print self.robo_pos
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        if (not ((np.isnan(self.hum_pos)).any() or (np.isnan(self.hum_vel)).any())):
            #TODO: Propagate the person's state forward in time to current time

            #distance from the robot
            person_to_rob = self.hum_pos - self.robo_pos
            self.dist = np.linalg.norm(person_to_rob)

            #magnitude of velocity in the direction of the robot
            self.vel_mag = np.dot(self.hum_vel, person_to_rob)/self.dist

            if self.stop_human:
                self.stop_human = not self.can_go()
            else:
                self.stop_human = self.should_stop()
        else:
            print 'No Human'
            self.stop_human = False
        
        #publish 
        stop_msg = BoolMsg()
        stop_msg.data = self.stop_human
        print stop_msg
        self.hum_signal_pub.publish(stop_msg)
        
        #visualize
        self.pub_visuals()
        return
    
    def can_go(self):
        min_dist = 3.
        if (self.dist>min_dist):
            return True
        #velocity and distance
        evals = self.vel_mag - self.slope * (self.dist - min_dist)

        if (evals>0):
            return True
        else:
            return False
    
    def should_stop(self):
        #distance threshold
        min_dist = 1.
        if (self.dist<min_dist):
            return True
        elif (self.dist>4.):
            return False

        #velocity and distance
        evals = self.vel_mag - self.slope * (self.dist - min_dist)
        if (evals>0):
            return True
        else:
            return False

    def pub_visuals(self):
        if self.stop_human:
            color = (1.,0.,0.,1.) #r,g,b
        else:
            color = (0.,1.,0.,1.) #r,g,b
        
        viz_ = Marker()
        viz_.header.stamp = self.cur_time
        viz_.header.frame_id = self.frame
        viz_.ns = 'human/safety/visuals'
        viz_.id = 1
        viz_.type = Marker.CYLINDER
        viz_.action = Marker.ADD

        viz_.pose.position.x = self.robo_pos[0]
        viz_.pose.position.y = self.robo_pos[1]
        viz_.pose.position.z = 1.5

        viz_.pose.orientation.x = 0.
        viz_.pose.orientation.y = 0.
        viz_.pose.orientation.z = 0.
        viz_.pose.orientation.w = 1.

        viz_.scale.x = 1.
        viz_.scale.y = 1.
        viz_.scale.z = .01

        viz_.color.r = color[0]
        viz_.color.g = color[1]
        viz_.color.b = color[2]
        viz_.color.a = color[3]

        viz_.lifetime = rospy.Duration()
        
        temp_markers = MarkerArray()
        temp_markers.markers.append(viz_)
        self.hum_signal_viz.publish(temp_markers)
        
        
def main():
    rospy.init_node('human_safety_publisher')
    hum_pub = HumanSafetyPub()
    rospy.spin()

#MAIN
if __name__=='__main__':
    main()
