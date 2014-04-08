#! /usr/bin/python

import numpy as np
from scipy.spatial import KDTree
from collections import deque
import yaml
from threading import RLock

import roslib
roslib.load_manifest("cpl_collab_manip")
import rospy

import tf
from geometry_msgs.msg import PoseStamped, PoseArray
from actionlib import SimpleActionClient
from roslaunch.substitution_args import resolve_args

from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from hrl_geom.pose_converter import PoseConv
from ur_cart_move.traj_planner import TrajPlanner, pose_offset, pose_interp
from ur_cart_move.spline_traj_executor import SplineTraj
from ur_cart_move.msg import SplineTrajAction, SplineTrajGoal
from ur_cart_move.ur_cart_move import ArmInterface, RAVEKinematics
from robotiq_c_model_control.robotiq_c_ctrl import RobotiqCGripper
from pykdl_utils.kdl_kinematics import create_kdl_kin
from ar_tag_manager_iface import create_slot_tree, ARTagManagerInterface

BIN_HEIGHT_DEFAULT = 0.1
TABLE_OFFSET_DEFAULT = -0.2
TABLE_CUTOFF_DEFAULT = 0.05
VEL_MULT = 4.7

class ARTagManager(ARTagManagerInterface):
    def __init__(self, bin_slots, available_bins=None, human_slots=range(3)):
        lifecam_kin = create_kdl_kin('base_link', 'lifecam1_optical_frame')
        self.camera_pose = lifecam_kin.forward([])

        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, 
                                       self.ar_cb, queue_size=1)
        self.clean_mkrs_pub = rospy.Publisher("/ar_pose_marker_clean", AlvarMarkers)
        self.ar_count = 0

    def ar_cb(self, msg):
        if self.lock.acquire(blocking=0):
            cur_time = rospy.get_time()
            for marker in msg.markers:
                marker.pose.header = marker.header
                if marker.id not in self.ar_poses:
                    self.ar_poses[marker.id] = deque()
                if len(self.ar_poses[marker.id]) == self.filter_size:
                    self.ar_poses[marker.id].popleft()
                self.ar_poses[marker.id].append([cur_time, marker.pose])
            # republish cleaned markers
            if self.ar_count == 10:
                bin_data = self.get_all_bin_poses()
                new_msg = AlvarMarkers()
                for bid in self.ar_poses:
                    mkr = AlvarMarker()
                    mkr.id = bid
                    ar_pose = PoseConv.to_homo_mat(bin_data[bid][:2])
                    mkr.pose = PoseConv.to_pose_stamped_msg("/lifecam1_rgb_optical_frame", 
                                                            self.camera_pose**-1 * ar_pose)
                    new_msg.markers.append(mkr)
                self.clean_mkrs_pub.publish(new_msg)
                self.ar_count = 0
            self.ar_count += 1
            self.lock.release()

def main():
    pass

if __name__ == "__main__":
    main()
