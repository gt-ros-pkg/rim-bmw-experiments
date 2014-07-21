#! /usr/bin/python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped

from ellipsoid_space import EllipsoidSpace
from hrl_geom.pose_converter import PoseConv

def main():
    rospy.init_node('view_ellipsoidal_traj')
    e_space = EllipsoidSpace(1)
    while not rospy.is_shutdown():
        x_i, y_i, z_i = np.random.uniform(-2.5, 2.5, 3)
        x_f, y_f, z_f = np.random.uniform(-2.5, 2.5, 3)
        lat_i, lon_i, height_i = e_space.pos_to_ellipsoidal(x_i, y_i, z_i)
        lat_f, lon_f, height_f = e_space.pos_to_ellipsoidal(x_f, y_f, z_f)
        ell_pose_traj = e_space.create_ellipsoidal_path(
                (lat_i, lon_i, height_i), (0., 0., 0., 1.),
                (lat_f, lon_f, height_f), (0., 0., 0., 1.),
                0.01)
        cart_pose_traj = [e_space.ellipsoidal_to_pose(ell_pose) for ell_pose in ell_pose_traj]
        print len(cart_pose_traj)
        # print cart_pose_traj
        ell_pose_pub = rospy.Publisher("/ell_pose_viz", PoseStamped)
        r = rospy.Rate(100)
        for cart_pose in cart_pose_traj:
            if rospy.is_shutdown():
                return
            ell_pose_pub.publish(PoseConv.to_pose_stamped_msg('/base_link', cart_pose))
            r.sleep()

if __name__ == "__main__":
    main()
