#! /usr/bin/python

import numpy as np

import rospy
from hrl_geom.pose_converter import PoseConv
from geometry_msgs.msg import PoseStamped, PoseArray

def main():
    rospy.init_node('pub_pose')

    pub = rospy.Publisher('/test_pose', PoseStamped)

    pose = np.mat([[  3.09961088e-17,  8.62413258e-01,  5.06204872e-01,  1.58647222e+00],
                   [ -5.28075818e-17,  5.06204872e-01, -8.62413258e-01, -5.35922245e-01],
                   [ -1.00000000e+00,  0.00000000e+00,  6.12323400e-17,  2.34097324e+00],
                   [  0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])
    frame = '/base_link'

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(PoseConv.to_pose_stamped_msg(frame, pose))
        r.sleep()

if __name__ == "__main__":
    main()
