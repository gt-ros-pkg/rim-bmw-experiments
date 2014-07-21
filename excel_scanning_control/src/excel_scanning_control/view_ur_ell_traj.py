#! /usr/bin/python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

from ellipsoidal_control.ellipsoid_space import EllipsoidSpace
from hrl_geom.pose_converter import PoseConv
from ur_py_utils.arm_iface import ArmInterface
from ur_kin_py.kin import Kinematics

def main():
    rospy.init_node('view_ellipsoidal_traj')

    arm = ArmInterface()
    kin = Kinematics(Kinematics.UR10_ID)

    js_msg = JointState()
    js_msg.name = ArmInterface.JOINT_NAMES
    js_msg.position = [0.]*6
    js_msg.velocity = [0.]*6
    js_msg.effort = [0.]*6
    js_pub = rospy.Publisher("/joint_states", JointState)

    e_space = EllipsoidSpace(0.01)
    q_last = [0., 0., np.pi/2., 0., 0., 0.,]
    ell_wp_list = [(np.pi/2.+0.1, np.pi, 4.8),
                   (np.pi/2.-1.2, np.pi, 4.8),
                   (np.pi/2.-1.2, np.pi, 4.1),
                   (np.pi/2.-1.0, np.pi+0.7, 3.9),
                   (np.pi/2.-1.0, np.pi-0.7, 5.0)]
    ell_wp_ind = 0
    while not rospy.is_shutdown():
        if False:
            x_i, x_f = np.random.uniform(-0.5, 0.0, 2)
            y_i, y_f = np.random.uniform(-0.5, 0.5, 2)
            z_i, z_f = np.random.uniform(0.0, 0.5, 2)
            lat_i, lon_i, height_i = e_space.pos_to_ellipsoidal(x_i, y_i, z_i)
            lat_f, lon_f, height_f = e_space.pos_to_ellipsoidal(x_f, y_f, z_f)
        if ell_wp_ind == len(ell_wp_list)-1:
            return
        lat_i, lon_i, height_i = ell_wp_list[ell_wp_ind]
        lat_f, lon_f, height_f = ell_wp_list[ell_wp_ind+1]
        ell_wp_ind += 1
        ell_pose_traj = e_space.create_ellipsoidal_path(
                (lat_i, lon_i, height_i), (0., 0., 0., 1.),
                (lat_f, lon_f, height_f), (0., 0., 0., 1.),
                0.005)
        cart_pose_traj = [e_space.ellipsoidal_to_pose(ell_pose) for ell_pose in ell_pose_traj]
        print len(cart_pose_traj)
        # print cart_pose_traj
        ell_pose_pub = rospy.Publisher("/ell_pose_viz", PoseStamped)
        r = rospy.Rate(125)
        for cart_pose in cart_pose_traj:
            if rospy.is_shutdown():
                return
            homo_offset_pose = np.mat(np.eye(4))
            homo_offset_pose[0,3] = 1.2
            homo_cart_pose = PoseConv.to_homo_mat(cart_pose)
            # homo_cart_pose[0,3] += 0.5
            ell_pose_pub.publish(PoseConv.to_pose_stamped_msg('/base_link', cart_pose))
            q_sol = kin.inverse(homo_offset_pose*homo_cart_pose, q_last)
            if q_sol is not None:
                js_msg.position = q_sol
                q_last = q_sol
            js_msg.header.frame_id = '/base_link'
            js_msg.header.stamp = rospy.Time.now()
            js_pub.publish(js_msg)
            r.sleep()

if __name__ == "__main__":
    main()
