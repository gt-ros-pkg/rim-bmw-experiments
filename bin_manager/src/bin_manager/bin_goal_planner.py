#! /usr/bin/python

import numpy as np

import rospy
from geometry_msgs.msg import PoseStamped, PoseArray

from hrl_geom.pose_converter import PoseConv

class BinGoalPlanner(object):
    def __init__(self, ar_tag_man):
        # self.ar_tag_man = ARTagManager(bin_slots, available_bins)
        self.ar_tag_man = ar_tag_man

        self.place_offset = 0.020
        self.ar_offset = 0.100

        # calculate grasp height
        # ar poses give the top edge of the bin
        self.gripper_tip_end = 0.156 # end of rubber
        self.gripper_tip_start = 0.121 # beginning of rubber
        self.gripper_palm = 0.083 # where the metal inside starts
        # self.grasp_height = self.gripper_tip_start - 0.2 * (self.gripper_tip_start - self.gripper_palm)
        # self.grasp_height = self.gripper_tip_end + 0.050
        self.grasp_height = 0.15

        self.grasp_rot = 0.0

        self.cur_grasp_bin_id = None
        self.cur_ar_place_tag_pose = None

        self.pose_array_pub = rospy.Publisher('/ar_pose_array', PoseArray, latch=True)
        self.grasp_goal_pub = rospy.Publisher('/grasp_goal', PoseStamped, latch=True)
        self.place_goal_pub = rospy.Publisher('/place_goal', PoseStamped, latch=True)

    # given semanic bin/slot IDs, determine the necessary grasp pose
    # and place pose to complete the task
    def generate_bin_location_goals(self, grasp_bin_id, place_slot_id):
        ########################## create waypoints #############################
        ar_grasp_tag_pose = self.ar_tag_man.get_bin_pose(grasp_bin_id)
        if ar_grasp_tag_pose is None:
            rospy.loginfo('Failed getting grasp pose')
            return False
        # convert AR tag location to a gripper goal location
        grasp_offset = PoseConv.to_homo_mat(
                [0., self.ar_offset, self.grasp_height],
                [0., np.pi/2, self.grasp_rot])
        grasp_pose = PoseConv.to_homo_mat(ar_grasp_tag_pose) * grasp_offset

        if place_slot_id not in self.ar_tag_man.get_slot_ids():
            rospy.loginfo('Failed getting place pose')
            return False
        ar_place_tag_pose = self.ar_tag_man.get_bin_slot(place_slot_id)
        # convert AR tag location to a gripper goal location
        place_offset = PoseConv.to_homo_mat(
                [0., self.ar_offset, self.grasp_height + self.place_offset],
                [0., np.pi/2, self.grasp_rot])
        place_pose = PoseConv.to_homo_mat(ar_place_tag_pose) * place_offset

        self.cur_grasp_bin_id = grasp_bin_id
        self.cur_ar_place_tag_pose = ar_place_tag_pose

        self.grasp_goal_pub.publish(PoseConv.to_pose_stamped_msg('/base_link', grasp_pose))
        self.place_goal_pub.publish(PoseConv.to_pose_stamped_msg('/base_link', place_pose))

        return grasp_pose, place_pose

    # This sets the current bin location to be where we just placed it.
    # Since the arm's occulsion might keep the robot from finding the new AR
    # tag location, it just sets it to where it just placed it
    def update_bin_location(self):
        self.ar_tag_man.set_bin_location(self.cur_grasp_bin_id, self.cur_ar_place_tag_pose)

    def publish_bin_locs(self):
        pose_dict = self.ar_tag_man.get_all_bin_poses()
        poses = []
        for binid in pose_dict:
            poses.append(pose_dict[binid])

        pose_array = PoseArray()
        pose_array.header.stamp = rospy.Time.now()
        pose_array.header.frame_id = '/base_link'
        for pose in poses:
            pose_array.poses.append(PoseConv.to_pose_msg(pose))
        self.pose_array_pub.publish(pose_array)
    
def publish_poses(topic, poses):
    pose_array_pub = rospy.Publisher('/ar_pose_array', PoseArray)
    pose_array = PoseArray()
    pose_array.header.stamp = rospy.Time.now()
    pose_array.header.frame_id = '/base_link'
    for pose in poses:
        pose_array.poses.append(PoseConv.to_pose_msg(pose))
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pose_array_pub.publish(pose_array)
        r.sleep()

def main():
    rospy.init_node('bin_goal_planner_interface')
    from ar_tag_manager_iface import ARTagManagerSimulator, setup_ar_tag_man_simulator
    ar_man_sim, bin_slots, bin_locs = setup_ar_tag_man_simulator()
    right_slots = np.array(range(0, 7))
    left_slots = np.array(range(7, 11))
    bin_goal_planner = BinGoalPlanner(ar_man_sim)
    cur_bins = right_slots.copy()
    cur_slots = left_slots.copy()
    rand_bin, rand_bin_loc = ar_man_sim.get_random_bin(cur_bins)
    rand_slot, rand_slot_loc = ar_man_sim.get_random_empty_slot(cur_slots)
    grasp_pose, place_pose = bin_goal_planner.generate_bin_location_goals(rand_bin, rand_slot)
    all_poses = [rand_bin_loc, rand_slot_loc, grasp_pose, place_pose]
    publish_poses('test', all_poses)

if __name__ == "__main__":
    main()
