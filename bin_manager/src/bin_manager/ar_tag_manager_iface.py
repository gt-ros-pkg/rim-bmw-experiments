#! /usr/bin/python

import numpy as np
import yaml
from scipy.spatial import KDTree
from threading import RLock
from collections import deque

from roslaunch.substitution_args import resolve_args

from hrl_geom.pose_converter import PoseConv

#         self.ar_man = ARTagManager(bin_slots, available_bins)
#         ar_grasp_tag_pose, grasp_is_table = self.ar_man.get_bin_pose(ar_grasp_id)
#         if ar_place_id not in self.ar_man.get_slot_ids():
#         ar_place_tag_pos, ar_place_tag_rot, place_is_table = self.ar_man.get_bin_slot(ar_place_id)
#         self.ar_man.set_bin_location(ar_grasp_id, ar_place_tag_pose)
#         ar_grasp_tag_pose, grasp_is_table = self.ar_man.get_bin_pose(ar_id)
#         ar_tags = self.ar_man.get_available_bins()
#         ar_locs = self.ar_man.get_slot_ids()
#         ar_tags = self.ar_man.get_available_bins()
#         empty_slots = self.ar_man.get_empty_slots()
#         print self.ar_man.get_available_bins()
#         bin_data = ar_man.get_all_bin_poses()

def create_slot_tree(bin_slots):
    pos_data = np.zeros((len(bin_slots),3))
    for i, bid in enumerate(sorted(bin_slots.keys())):
        pos_data[i,:] = bin_slots[bid][0]
    return KDTree(pos_data)

class ARTagManagerInterface(object):
    def __init__(self, bin_slots, available_bins=None):
        # distance from marker to slot which can be considered unified
        self.ar_unification_thresh = 0.12
        # self.ar_unification_thresh = rospy.get_param("~ar_unification_thresh", 0.12)

        self.bin_slots = bin_slots
        self.available_bins = available_bins
        self.slot_tree = create_slot_tree(bin_slots)
        self.camera_pose = np.mat(np.eye(4))

        self.ar_poses = {}
        self.lock = RLock()

    # forces bin location from a pose in the base_link frame
    def set_bin_location(self, mid, pose, cur_time=0.):
        with self.lock:
            # cur_time = rospy.get_time()
            self.ar_poses[mid].clear()
            self.ar_poses[mid].append([cur_time, PoseConv.to_homo_mat(pose)])

    def get_available_bins(self):
        with self.lock:
            bins = []
            for mid in self.ar_poses:
                if (len(self.ar_poses[mid]) > 0 and 
                        (self.available_bins is None or mid in self.available_bins)):
                    bins.append(mid)
            return bins

    def clean_ar_pose(self, ar_pose):
        ar_pose_mat = self.camera_pose * PoseConv.to_homo_mat(ar_pose)
        ang = np.arctan2(ar_pose_mat[1,0], ar_pose_mat[0,0])
        return (ar_pose_mat[:3,3].T.A[0], 
                [0., 0., ang])

    def get_bin_pose(self, bin_id):
        with self.lock:
            if bin_id not in self.ar_poses:
                print "Bin ID %d not found!" % bin_id
                return None, None
            pos_list, rot_list = [], []
            for cur_time, pose in self.ar_poses[bin_id]:
                pos, rot = self.clean_ar_pose(pose)
                pos_list.append(pos)
                rot_list.append(rot)
            med_pos, med_rot = np.median(pos_list,0), np.median(rot_list,0)
            return (med_pos.tolist(), med_rot.tolist())

    def get_all_bin_poses(self):
        with self.lock:
            bin_data = {}
            for bin_id in self.get_available_bins():
                bin_pose = self.get_bin_pose(bin_id)
                bin_data[bin_id] = [bin_pose[0], bin_pose[1]]
            return bin_data

    def get_bin_slot(self, slot_id):
        return self.bin_slots[slot_id]

    def get_slot_ids(self):
        return sorted(self.bin_slots.keys())

    def get_bin_slot_states(self):
        bin_poses = self.get_all_bin_poses()
        bin_ids = sorted(bin_poses.keys())
        bin_pos_data = np.array([bin_poses[bin_id][0] for bin_id in bin_ids])
        if len(bin_pos_data) != 0:
            dists, inds = self.slot_tree.query(bin_pos_data, k=1, 
                                               distance_upper_bound=self.ar_unification_thresh)
        else:
            dists, inds = [], []

        slot_states = [-1] * len(self.slot_tree.data)
        missing_bins = []
        for i, ind in enumerate(inds):
            bin_id = bin_ids[i]
            if ind == len(slot_states):
                missing_bins.append(bin_id)
                continue
            slot_states[ind] = bin_id
        return slot_states, missing_bins

    def get_filled_slots(self, slots_to_check=None, invert_set=False):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_bin_slot_states()
        bins = []
        for ind, slot_state in enumerate(slot_states):
            if slot_state != -1:
                slot_in_set = ind in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    bins.append(slot_state)
        return sorted(bins)

    # finds the closest empty slot to the pos position
    def get_empty_slot(self, slots_to_check=None, invert_set=False, pos=[0.,0.,0.]):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_bin_slot_states()
        dists, inds = self.slot_tree.query(pos, k=len(self.slot_tree.data)) 
        for ind in inds:
            if slot_states[ind] == -1:
                slot_in_set = ind in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    return self.get_slot_ids()[ind]
        return -1

    def get_empty_slots(self, slots_to_check=None, invert_set=False):
        if slots_to_check is None:
            slots_to_check = self.bin_slots.keys()
        slot_states, _ = self.get_bin_slot_states()
        slot_ids = self.get_slot_ids()
        empty_slots = []
        for ind, slot_state in enumerate(slot_states):
            if slot_state == -1:
                slot_in_set = ind in slots_to_check
                if np.logical_xor(not slot_in_set, not invert_set):
                    empty_slots.append(slot_ids[ind])
        return empty_slots

    def get_random_empty_slot(self, slots_to_check=None, invert_set=False):
        empty_slots = self.get_empty_slots(slots_to_check, invert_set)
        rand_slot = empty_slots[np.random.randint(len(empty_slots))]
        rand_slot_loc = self.get_bin_slot(rand_slot)
        return rand_slot, rand_slot_loc

    def get_random_bin(self, slots_to_check=None, invert_set=False):
        bins = self.get_filled_slots(slots_to_check, invert_set)
        rand_bin = bins[np.random.randint(len(bins))]
        rand_bin_loc = self.get_bin_pose(rand_bin)
        return rand_bin, rand_bin_loc

class ARTagManagerSimulator(ARTagManagerInterface):
    def __init__(self, bin_slots, bin_init_locs, available_bins=None):
        super(ARTagManagerSimulator, self).__init__(
                bin_slots, available_bins=None)
        for i in bin_init_locs:
            self.ar_poses[i] = deque()
            self.ar_poses[i].append([0., bin_init_locs[i]])

def load_bin_slots(filename):
    f = file(resolve_args(filename), 'r')
    bin_slots = yaml.load(f)['data']
    f.close()
    return bin_slots

def setup_ar_tag_man_simulator():
    bin_slots = load_bin_slots('$(find bin_manager)/test/test_bin_slots.yaml')
    bin_locs = load_bin_slots('$(find bin_manager)/test/test_bin_locs.yaml')
    ar_man_sim = ARTagManagerSimulator(bin_slots, bin_locs)
    return ar_man_sim, bin_slots, bin_locs

def main():
    ar_man_sim, bin_slots, bin_locs = setup_ar_tag_man_simulator()

    empty_slots = ar_man_sim.get_empty_slots()
    print "Current slot states:", ar_man_sim.get_bin_slot_states()
    print "Empty slots:", empty_slots
    rand_slot, rand_slot_loc = ar_man_sim.get_random_empty_slot()
    available_bins = ar_man_sim.get_available_bins()
    print "Bins available:", available_bins
    rand_bin, rand_bin_loc = ar_man_sim.get_random_bin()

    print "Moving bin", rand_bin, "at location", rand_bin_loc
    print "to slot", rand_slot, "at location", rand_slot_loc
    ar_man_sim.set_bin_location(rand_bin, rand_slot_loc)
    print "New slot states:", ar_man_sim.get_bin_slot_states()

if __name__=="__main__":
    main()
