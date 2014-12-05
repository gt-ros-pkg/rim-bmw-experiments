#!/usr/bin/env python

# Publishes whether it is safe for the robot to move

import roslib
roslib.load_manifest('bmw_percep')

import time
import rospy
import tf
import matplotlib.pyplot as plt
import matplotlib.axis as m_axes

from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Bool as BoolMsg
from math import sqrt
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
import threading

from excel_servers.srv import KinematicsInfo

class HumanSafetyPub:
    def __init__(self):

        #plt.axis([0., 6., 0., 6.])

        #plt.axis([)
        #plt.show(1)
        self.thread_plot = threading.Thread(target=self.plot_things)


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
        self.started_plot = False
        
        self.hys_reset = 3.

        self.robo_frame = "base_link"
        self.robo_ee_frame = "wrist_3_link"
        self.slope = 2
        self.min_dist1 = .75#1.25
        self.min_dist2 = 1.25#1.75

        self.min_dist_ee_go = 1.1
        self.min_dist_base_go = 1.1
        self.min_dist_ee_stop = 0.75
        self.min_dist_base_stop = 0.75

        #initialize to don't stop human
        self.stop_human = False
        self.in_hysterisis_start = rospy.Time.now()
        self.in_hysterisis = False
        self.stop_human_base = False
        self.stop_human_ee = False
        
        self.kinematics_info = rospy.ServiceProxy('get_kinematics_info', KinematicsInfo)
	
        kinematics = self.kinematics_info()
        joint_positions = kinematics.positions.data
        joint_velocities = kinematics.velocities.data
        joint_jacobian = kinematics.jacobian.data
        
       

        
    #callback receives person position and velocity
    def pers_cb(self, msg):
        try:
            human_safety_mode = rospy.get_param("human_safety_mode")
        except KeyError:
            human_safety_mode = 1
        
        print human_safety_mode
        
        
        kinematics = self.kinematics_info()
        joint_positions = kinematics.positions.data
        joint_velocities = kinematics.velocities.data
        joint_jacobian = kinematics.jacobian.data

        
	
        self.frame = msg.header.frame_id
        self.hum_time = msg.header.stamp
        self.hum_pos = np.array([msg.poses[0].position.x, msg.poses[0].position.y])
        self.hum_vel = np.array([msg.poses[1].position.x, msg.poses[1].position.y])
        self.cur_time_tf = rospy.Time(0)
        self.cur_time = rospy.Time.now()

        
        

        #Get robot transform
        try:
            self.robo_listener.waitForTransform(self.frame, self.robo_frame, 
                                                self.cur_time_tf,
                                                rospy.Duration(1./30.))
            (trans_c, rot) = self.robo_listener.lookupTransform(self.frame, 
                                                              self.robo_frame, 
                                                              self.cur_time_tf)
            (trans_ee, rot) = self.robo_listener.lookupTransform(self.frame, 
                                                              self.robo_ee_frame, 
                                                              self.cur_time_tf)
             
                        
            #by default, robo-pos is center
            self.robo_pos = np.array([trans_c[0], trans_c[1]])
            self.robo_ee_pos = np.array([trans_ee[0], trans_ee[1]])
            self.robo_ee_vel = np.array([0.0,0.0])
            if human_safety_mode == 3:
                temp  = np.dot(np.resize(np.asarray(joint_jacobian),(6,7)), np.asarray(joint_velocities))
                
                self.robo_ee_vel[0]  = temp[0]
                self.robo_ee_vel[1]  = temp[1]
                print self.robo_ee_vel
            
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return

        if (not ((np.isnan(self.hum_pos)).any() or (np.isnan(self.hum_vel)).any())):
            dist_center = np.linalg.norm(np.array(self.hum_pos - [trans_c[0], trans_c[1]]))
            dist_ee = np.linalg.norm(np.array(self.hum_pos - [trans_ee[0], trans_ee[1]]))

            #TODO: Propagate the person's state forward in time to current time

            #distance from the robot
            person_to_ee = -self.hum_pos +  self.robo_ee_pos
            self.dist2ee = dist_ee
            
            self.dist2base = dist_center
                
            
            #magnitude of velocity in the direction of the robot
            if human_safety_mode == 1:
                self.vel_mag = np.dot(self.hum_vel, person_to_ee)/self.dist2ee
            elif human_safety_mode == 3:
                self.vel_mag = np.dot((self.hum_vel-self.robo_ee_vel), person_to_ee)/self.dist2ee

            if self.stop_human:
                self.should_stop_ee()
                if self.should_stop_base():
                    self.in_hysterisis  = False
                elif self.stop_human_base:
                    if (not self.in_hysterisis):
                        self.in_hysterisis = True
                        self.in_hysterisis_start = self.cur_time
                    else:
                        self.min_dist_base_go = self.min_dist_base_go - (self.cur_time-self.in_hysterisis_start).to_sec()/10
                        if self.min_dist_base_go < (self.min_dist_base_stop + 0.1):
                            self.min_dist_base_go = (self.min_dist_base_stop + 0.1)
                                
                if self.stop_human_base and (not self.stop_human_ee):               
                    self.stop_human = not self.can_go_base()
                elif self.stop_human_ee and (not self.stop_human_base):
                    self.stop_human = not self.can_go_ee()
                else:
                    self.stop_human = not (self.can_go_ee() and self.can_go_base())
                    
            else:
                self.stop_human = self.should_stop_ee() or self.should_stop_base()

            #visualize
            if(not self.started_plot):
                self.thread_plot.start()
                self.started_plot = True
     
        else:
            #print 'No Human'
            self.stop_human = False
        
        #publish 
        stop_msg = BoolMsg()
        stop_msg.data = self.stop_human
        #print stop_msg
        self.hum_signal_pub.publish(stop_msg)
        
        # self.plot_things()
        self.pub_visuals()
        
        
        return

    def plot_things(self):
        plt.ion()
        self.fig = plt.figure(figsize=(13,13))

        plt.hold(True)

        xylims = [0., 6., -2., 2.]

        framy=0
        
        while not rospy.is_shutdown():
            
            #plot the lines first
            line1x = [(self.min_dist_ee_stop-10.0), (self.min_dist_ee_stop+20.0)]
            line1y = [(self.slope*(-10.0)), (self.slope*(20.0))]
            line2x = [(self.min_dist_ee_go-10.0), (self.min_dist_ee_go+20.0)]
            line2y = [(self.slope*(-10.0)), (self.slope * (20.0))]
            
            fixl1x = [self.min_dist_base_stop, self.min_dist_base_stop]
            fixl1y = [-10, 10]
            
            fixl2x = [self.min_dist_base_go, self.min_dist_base_go]
            fixl2y = [-10., 10]
            
            #fix
            
            pointx = self.dist2ee
            pointy = self.vel_mag
        
            #if framy is 0:
            plt.clf()
                
            #framy = (framy+1)%3

            plt.axis(xylims, figure=self.fig)
            plt.subplot(211)
            if self.stop_human_ee:
                plt.plot(line1x, line1y,'r', line2x, line2y, 'b', pointx, pointy, 'ro', markersize=20)
            else:
                plt.plot(line1x, line1y,'r', line2x, line2y, 'b', pointx, pointy, 'go', markersize=20)
            plt.ylim(-2,3)
            plt.xlim(0,3)
            plt.subplot(212)
            if self.stop_human_base:
                plt.plot(self.dist2base, 0,'ro', fixl1x, fixl1y, 'r', fixl2x, fixl2y, 'b', markersize=20)
            else:
                plt.plot(self.dist2base, 0,'go', fixl1x, fixl1y, 'r', fixl2x, fixl2y, 'b', markersize=20)
            plt.xlim(0,3)
            plt.ylim(-1,1)
            plt.draw()
            time.sleep(.03)
        #plt.close()
        

    def can_go_ee(self):
        
        #velocity and distance
        evals = self.vel_mag - self.slope * (self.dist2ee - self.min_dist_ee_go)

        if (evals>0):
            return False
        else:
            self.stop_human_ee = False
            return True
            
    def can_go_base(self):

        if (self.dist2base < self.min_dist_base_go):
            return False
        else:
            self.stop_human_base = False
            return True
    
    def should_stop_ee(self):
        #velocity and distance
        evals = self.vel_mag - self.slope * (self.dist2ee - self.min_dist_ee_stop)
        if (evals>0):
            self.stop_human_ee = True
            return True
        else:
            return False
    
    def should_stop_base(self):
        self.min_dist_base_go = 1.1
        if (self.dist2base<self.min_dist_base_stop):
            self.stop_human_base = True
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

        viz_.scale.x = self.min_dist1
        viz_.scale.y = self.min_dist1
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
