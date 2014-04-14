#! /usr/bin/python

import numpy as np
from scipy.spatial import KDTree
from collections import deque
import yaml
from threading import RLock

import rospy

import tf
from geometry_msgs.msg import PoseStamped, PoseArray
from actionlib import SimpleActionClient
from roslaunch.substitution_args import resolve_args

from ar_track_alvar.msg import AlvarMarkers, AlvarMarker
from hrl_geom.pose_converter import PoseConv
# from pykdl_utils.kdl_kinematics import create_kdl_kin
from ar_tag_manager_iface import ARTagManagerInterface, load_bin_slots

BIN_HEIGHT_DEFAULT = 0.1
TABLE_OFFSET_DEFAULT = -0.2
TABLE_CUTOFF_DEFAULT = 0.05
VEL_MULT = 4.7

class ARTagManager(ARTagManagerInterface):
    def __init__(self, bin_slots, available_bins=None):
        super(ARTagManager, self).__init__(
                bin_slots, available_bins=None)
        # lifecam_kin = create_kdl_kin('base_link', 'lifecam1_optical_frame')
        # self.camera_pose = lifecam_kin.forward([])
        camera_pos = [1.219, -1.149, 1.591]
        camera_quat = [0.929, 0.261, -0.104, -0.240]
        self.table_height = -0.098
        self.bin_height = 0.101
        self.camera_pose = PoseConv.to_homo_mat(camera_pos, camera_quat)

        self.filter_size = 6

        self.clean_period = 0.1
        self.last_clean_pub = 0.

        self.ar_sub = rospy.Subscriber("/ar_pose_marker", AlvarMarkers, 
                                       self.ar_cb, queue_size=1)

    def ar_cb(self, msg):
        if self.lock.acquire(blocking=0):
            cur_time = rospy.get_time()
            now_time = rospy.Time.now()
            for marker in msg.markers:
                marker.pose.header = marker.header
                if marker.id not in self.ar_poses:
                    self.ar_poses[marker.id] = deque()
                if len(self.ar_poses[marker.id]) == self.filter_size:
                    self.ar_poses[marker.id].popleft()
                self.ar_poses[marker.id].append([cur_time, marker.pose])
            # republish cleaned markers
            if cur_time - self.last_clean_pub > self.clean_period:
                self.last_clean_pub = cur_time
                bin_data = self.get_all_bin_poses()
                pose_arr = PoseArray()
                frame = "/prosilica1_optical_frame"
                # print bin_data
                for bid in self.ar_poses:
                    ar_pose = PoseConv.to_homo_mat(bin_data[bid][:2])
                    pose_msg = PoseConv.to_pose_msg(self.camera_pose**-1 * ar_pose)
                    pose_arr.poses.append(pose_msg)
                pose_arr.header.frame_id = frame
                pose_arr.header.stamp = now_time
                self.clean_mkrs_pub.publish(pose_arr)
            self.lock.release()

    # forces bin location from a pose in the base_link frame
    def set_bin_location(self, mid, pose, cur_time=0.):
        super(ARTagManager, self).set_bin_location(
                mid, self.camera_pose**-1 * PoseConv.to_homo_mat(pose), cur_time)

        # def get_bin_pose(self, bin_id):
        #     bin_pos, bin_rot = super(ARTagManager, self).get_bin_pose(bin_id)
        #     bin_pos[2] = self.table_height + self.bin_height + 0.8 + .9
        #     return bin_pos, bin_rot

        # def filter_pose(self, in_pos, in_rot):
        #     in_pos[2] = self.table_height + self.bin_height + 0.8 + .9
        #     return in_pos, in_rot

        # def get_all_bin_poses(self):
        #     with self.lock:
        #         bin_data = {}
        #         for bin_id in self.get_available_bins():
        #             bin_pose = self.get_bin_pose(bin_id)
        #             print bin_pose
        #             bin_data[bin_id] = [bin_pose[0], bin_pose[1]]
        #         return bin_data

def main():
    rospy.init_node('ar_tag_manager')

    if False:
        # SAVE POSES
        ar_tag_man = ARTagManager({})
        rospy.sleep(4.)
        bin_data = ar_tag_man.get_all_bin_poses()
        f = file('ar_poses.yaml', 'w')
        for bid in bin_data:
            print bid, bin_data[bid]
        yaml.dump({'data':bin_data}, f)
        f.close()

        poses_pub = rospy.Publisher('/ar_pose_array', PoseArray)
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            poses = PoseArray()
            poses.header.stamp = rospy.Time.now()
            poses.header.frame_id = '/base_link'
            for bid in bin_data:
                poses.poses.append(PoseConv.to_pose_msg(bin_data[bid][:2]))
            poses_pub.publish(poses)
            r.sleep()
        return

    bin_slots = load_bin_slots('$(find bin_manager)/src/bin_manager/bin_slots_both1.yaml')
    print bin_slots
    ar_tag_man = ARTagManager(bin_slots)
    rospy.sleep(3.)
    print ar_tag_man.get_all_bin_poses()
    rospy.spin()

    empty_slots = ar_tag_man.get_empty_slots()
    print "Current slot states:", ar_tag_man.get_bin_slot_states()
    print "Empty slots:", empty_slots
    rand_slot, rand_slot_loc = ar_tag_man.get_random_empty_slot()
    available_bins = ar_tag_man.get_available_bins()
    print "Bins available:", available_bins
    rand_bin, rand_bin_loc = ar_tag_man.get_random_bin()

    print "Moving bin", rand_bin, "at location", rand_bin_loc
    print "to slot", rand_slot, "at location", rand_slot_loc
    ar_tag_man.set_bin_location(rand_bin, rand_slot_loc)
    print "New slot states:", ar_tag_man.get_bin_slot_states()

if __name__ == "__main__":
    main()
