#! /usr/bin/python

import numpy as np
from scipy.spatial import KDTree
from collections import deque
import yaml

import rospy

import tf
from geometry_msgs.msg import PoseStamped, PoseArray
from actionlib import SimpleActionClient
from roslaunch.substitution_args import resolve_args

from hrl_geom.pose_converter import PoseConv

from trajectory_gen.traj_planner import TrajPlanner, pose_offset, pose_interp
from trajectory_gen.spline_traj_executor import SplineTraj
from ur_cart_move.msg import SplineTrajAction, SplineTrajGoal

from ur_cart_move.ur_cart_move import ArmInterface, RAVEKinematics
from robotiq_c_model_control.robotiq_c_ctrl import RobotiqCGripper

BIN_HEIGHT_DEFAULT = 0.1
TABLE_OFFSET_DEFAULT = -0.2
TABLE_CUTOFF_DEFAULT = 0.05
VEL_MULT = 4.7

class BinManager(object):
    def __init__(self, arm_prefix, bin_slots, available_bins=None):

        self.waypt_offset = rospy.get_param("~waypt_offset", 0.25)
        self.waypt_robot_min_dist = rospy.get_param("~waypt_robot_min_dist", -0.70)

        sim_prefix = rospy.get_param("~sim_arm_prefix", "/sim2")
        self.arm_cmd = ArmInterface(timeout=0., topic_prefix=arm_prefix)
        self.arm_sim = ArmInterface(timeout=0., topic_prefix=sim_prefix)
        if arm_prefix not in ["", "/"]:
            self.gripper = None
        else:
            self.gripper = RobotiqCGripper()

        self.ar_tag_man = ARTagManager(bin_slots, available_bins)
        self.pose_pub = rospy.Publisher('/test', PoseStamped)
        #self.move_bin = rospy.ServiceProxy('/move_bin', MoveBin)
        self.traj_as = SimpleActionClient('spline_traj_as', SplineTrajAction)

        print 'Waiting for trajectory AS...', 
        self.traj_as.wait_for_server()
        print 'found.'
        if self.gripper is not None:
            print 'Waiting for gripper...', 
            self.gripper.wait_for_connection()
            print 'found.'
        print 'Waiting for arms...', 
        if (not self.arm_cmd.wait_for_states(timeout=1.) or
            not self.arm_sim.wait_for_states(timeout=1.)):
            print 'Arms not connected!'
        print 'found.'
        self.update_payload(empty=True)

    def update_payload(self, empty):
        if empty:
            self.arm_cmd.set_payload(pose=([0., -0.0086, 0.0353], [0.]*3), payload=0.89)
        else:
            self.arm_cmd.set_payload(pose=([-0.03, -0.0086, 0.0453], [0.]*3), payload=1.40)

    def execute_traj(self, traj):
        goal = SplineTrajGoal(traj=traj.to_trajectory_msg())
        self.arm_cmd.unlock_security_stop()
        self.traj_as.wait_for_server()
        self.traj_as.send_goal(goal)
        self.traj_as.wait_for_result()
        result = self.traj_as.get_result()
        rospy.loginfo("Trajectory result:" + str(result))
        return result.success, result.is_robot_running

    def move_bin(self, grasp_bin_id, place_slot_id):
        # bin_pose
        # move to pregrasp, above the bin (traversal)
        # move down towards the bin
        # grasp the bin
        # lift up
        # move to preplace (traversal)
        # move down
        # release bin

        mid_pts = []
        if np.logical_xor(place_is_table, grasp_is_table):
            waypt_line = [0., 1., self.waypt_offset]
            grasp_pt = [grasp_pose[0,3], grasp_pose[1,3], 1.]
            place_pt = [place_pose[0,3], place_pose[1,3], 1.]
            waypt_pt = np.cross(waypt_line, np.cross(grasp_pt, place_pt))
            waypt_pt /= waypt_pt[2]
            waypt_pt[0] = min(waypt_pt[0], self.waypt_robot_min_dist)
            waypt = pose_interp(pregrasp_pose, preplace_pose, 0.5)
            waypt[:3,3] = np.mat([waypt_pt[0], waypt_pt[1], 
                                  self.grasp_height+self.bin_height+self.grasp_lift]).T
            mid_pts.append(waypt)

        if False:
            waypts = (  [pregrasp_pose, grasp_pose, pregrasp_pose] 
                      + mid_pts 
                      + [preplace_pose, place_pose, preplace_pose])
            r = rospy.Rate(1)
            for pose in waypts:
                self.pose_pub.publish(PoseConv.to_pose_stamped_msg("/base_link", pose))
                r.sleep()
            print (pregrasp_pose, grasp_pose, pregrasp_pose, mid_pts, 
                   preplace_pose, place_pose, preplace_pose)

        #########################################################################
        # execution

        grasp_traj = self.plan_grasp_traj(pregrasp_pose, grasp_pose)
        if grasp_traj is None:
            return False
        success, is_robot_running = self.execute_traj(grasp_traj)
        if not success:
            rospy.loginfo('Failed on grasping bin')
            return False
        if self.gripper is not None:
            self.gripper.close(block=True)
            self.update_payload(empty=False)
        move_traj = self.plan_move_traj(pregrasp_pose, mid_pts, preplace_pose, place_pose)
        if move_traj is None:
            return False
        success, is_robot_running = self.execute_traj(move_traj)
        if not success:
            rospy.loginfo('Failed on moving bin')
            return False
        if self.gripper is not None:
            self.gripper.goto(0.042, 0., 0., block=False)
            rospy.sleep(0.5)
            self.update_payload(empty=True)
        retreat_traj = self.plan_retreat_traj(preplace_pose, place_pose)
        if retreat_traj is None:
            return False
        success, is_robot_running = self.execute_traj(retreat_traj)
        if not success:
            rospy.loginfo('Failed on retreating from bin')
            return False

        # cleanup
        return True

    def system_reset(self):
        home_traj = self.plan_home_traj()
        self.execute_traj(home_traj)
        if self.gripper is not None:
            if self.gripper.is_reset():
                self.gripper.reset()
                self.gripper.activate()
        self.update_payload(empty=True)
        if self.gripper is not None:
            if self.gripper.get_pos() != 0.042:
                self.gripper.goto(0.042, 0., 0., block=False)

    def do_random_move_test(self):
        raw_input("Move to home")
        reset = True
        while not rospy.is_shutdown():
            if reset:
                self.system_reset()
            #raw_input("Ready")
            ar_tags = self.ar_tag_man.get_available_bins()
            ar_locs = self.ar_tag_man.get_slot_ids()
            grasp_tag_num = ar_tags[np.random.randint(0,len(ar_tags))]
            place_tag_num = ar_locs[np.random.randint(0,len(ar_locs))]
            if not self.move_bin(grasp_tag_num, place_tag_num):
                reset = True
                print 'Failed moving bin from %d to %d' % (grasp_tag_num, place_tag_num)

    def do_move_demo(self):
        reset = True
        while not rospy.is_shutdown():
            if reset:
                raw_input("Reset system needed")
                self.system_reset()
                reset = False
                raw_input("Ready?")
            ar_tags = self.ar_tag_man.get_available_bins()
            grasp_tag_num = ar_tags[np.random.randint(0,len(ar_tags))]
            empty_slots = self.ar_tag_man.get_empty_slots()
            place_tag_num = empty_slots[np.random.randint(0,len(empty_slots))]
            if not self.move_bin(grasp_tag_num, place_tag_num):
                reset = True
                print 'Failed moving bin from %d to %d' % (grasp_tag_num, place_tag_num)

    def do_above_demo(self):
        reset = True
        while not rospy.is_shutdown():
            if reset:
                self.system_reset()
                reset = False
            print "Available bins:"
            print self.ar_tag_man.get_available_bins()
            bin_id = int(raw_input("Bin ID:"))
            if not self.move_above_bin(bin_id):
                reset = True
                print "Failed", bin_id

    def do_plan_demo(self, plan):
        print plan
        raw_input("Reset system needed")
        self.system_reset()
        raw_input("Ready?")

        for grasp_tag_num, place_tag_num in plan:
            if rospy.is_shutdown():
                break

            if not self.move_bin(grasp_tag_num, place_tag_num):
                print 'Failed moving bin from %d to %d' % (grasp_tag_num, place_tag_num)
                return

def main():
    np.set_printoptions(precision=4)
    rospy.init_node("bin_manager")

    from optparse import OptionParser
    p = OptionParser()
    p.add_option('-f', '--file', dest="filename", 
                 default="$(find cpl_collab_manip)/config/bin_locs.yaml",
                 help="YAML file of bin locations.")
    p.add_option('-s', '--save', dest="is_save",
                 action="store_true", default=False,
                 help="Save ar tag locations to file.")
    p.add_option('-t', '--test', dest="is_test",
                 action="store_true", default=False,
                 help="Test robot in simulation moving bins around.")
    p.add_option('-d', '--demo', dest="is_demo",
                 action="store_true", default=False,
                 help="Test robot in reality moving a bin around.")
    p.add_option('-a', '--ademo', dest="is_ademo",
                 action="store_true", default=False,
                 help="Move above bins.")
    p.add_option('-p', '--plan', dest="is_plan",
                 action="store_true", default=False,
                 help="Move above bins.")
    p.add_option('-v', '--visualize', dest="is_viz",
                 action="store_true", default=False,
                 help="Visualize poses.")
    (opts, args) = p.parse_args()

    if opts.is_save:
        ar_tag_man = ARTagManager({})
        rospy.sleep(4.)
        bin_data = ar_tag_man.get_all_bin_poses()
        f = file(resolve_args(opts.filename), 'w')
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
    elif opts.is_test:
        f = file(resolve_args(opts.filename), 'r')
        bin_slots = yaml.load(f)['data']
        f.close()
        arm_prefix = "/sim1"
        bm = BinManager(arm_prefix, bin_slots)
        bm.do_random_move_test()
    elif opts.is_demo:
        f = file(resolve_args(opts.filename), 'r')
        bin_slots = yaml.load(f)['data']
        f.close()
        arm_prefix = ""
        bm = BinManager(arm_prefix, bin_slots)
        bm.do_move_demo()
    elif opts.is_ademo:
        f = file(resolve_args(opts.filename), 'r')
        bin_slots = yaml.load(f)['data']
        f.close()
        arm_prefix = ""
        bm = BinManager(arm_prefix, bin_slots)
        bm.do_above_demo()
    elif opts.is_plan:
        f = file(resolve_args(opts.filename), 'r')
        bin_slots = yaml.load(f)['data']
        f.close()
        arm_prefix = ""
        bm = BinManager(arm_prefix, bin_slots)
        plan = [(3,2), (11,1), (2,0), (3,5), (15,2)]
        bm.do_plan_demo(plan)
    elif opts.is_viz:
        f = file(resolve_args(opts.filename), 'r')
        bin_data = yaml.load(f)['data']
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
    else:
        print 'h'

    
    #bm = BinManager()
    #bm.do_random_move_test()
    #rospy.spin()

if __name__ == "__main__":
    if False:
        import cProfile
        cProfile.run('main()', 'bm_prof')
    else:
        main()
