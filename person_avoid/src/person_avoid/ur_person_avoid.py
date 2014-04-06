#! /usr/bin/python

import numpy as np

import rospy

from bin_manager.bin_man_motion_plan import BinManagerMotionPlanner
from person_avoid import avoid_traj
from hrl_geom.pose_converter import PoseConv
from hrl_geom.transformations import quaternion_slerp

class PersonAvoidBinManMotionPlanner(BinManagerMotionPlanner):
    def __init__(self, arm, kin):
        super(PersonAvoidBinManMotionPlanner, self).__init__(arm, kin)

    def create_midpt_traj(self, prev_t, prev_q, above_goal_pose):
        t_midpt_knots = []
        q_midpt_knots = []
        prev_pose = self.kin.forward(prev_q)
        prev_pos, prev_quat = PoseConv.to_pos_quat(prev_pose)
        goal_pos, goal_quat = PoseConv.to_pos_quat(above_goal_pose)

        # create midpts
        distmove = 0.1
        midptfrac1 = 0.5
        midptfrac2 = 0.8
        wayptvel = self.pregrasp_vel
        t_traj_knots, x_traj_knots, spline_traj = avoid_traj(prev_pos[:2], goal_pos[:2], 
                                                     distmove, midptfrac1, midptfrac2, wayptvel)
        waypt1_pos = [x_traj_knots[1][0], x_traj_knots[1][1], prev_pos[2]]
        waypt2_pos = [x_traj_knots[2][0], x_traj_knots[2][1], goal_pos[2]]
        waypt1_quat = quaternion_slerp(prev_quat, goal_quat, midptfrac1)
        waypt2_quat = quaternion_slerp(prev_quat, goal_quat, midptfrac2)
        waypt1_pose = PoseConv.to_homo_mat(waypt1_pos, waypt1_quat)
        waypt2_pose = PoseConv.to_homo_mat(waypt2_pos, waypt2_quat)
        mid_pts = [waypt1_pose, waypt2_pose]

        for mid_pose in mid_pts + [above_goal_pose]: 
            q_mid_pose = self.kin.inverse(mid_pose, prev_q,
                                          q_min=self.q_min, q_max=self.q_max)
            if q_mid_pose is None:
                print 'failed move to replace'
                print mid_pose
                print q_knots
                return None
            q_midpt_knots.append(q_mid_pose)
            # use a heuristic time based on the velocity to select a knot time
            dist = np.linalg.norm(mid_pose[:3,3]-prev_pose[:3,3])
            cur_t_delta = max(dist / self.pregrasp_vel, 0.3)
            t_midpt_knots.append(prev_t + cur_t_delta)
            prev_t += cur_t_delta
            prev_q = q_mid_pose
            prev_pose = mid_pose
        return t_midpt_knots, q_midpt_knots

def main():
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    import actionlib

    from ur_py_utils.arm_iface import ArmInterface
    from ur_py_utils.ur_controller_manager import URControllerManager
    from bin_manager.ar_tag_manager_iface import ARTagManagerSimulator, setup_ar_tag_man_simulator
    from ur_kin_py.kin import Kinematics

    from bin_manager.bin_goal_planner import BinGoalPlanner

    ar_man_sim, bin_slots, bin_locs = setup_ar_tag_man_simulator()
    right_slots = np.array(range(0, 7))
    left_slots = np.array(range(7, 11))

    rospy.init_node("ur_person_avoid")
    cman = URControllerManager()
    cman.start_joint_controller('pva_trajectory_ctrl')
    act_cli = actionlib.SimpleActionClient(
            '/pva_trajectory_ctrl/follow_joint_trajectory', 
            FollowJointTrajectoryAction)
    if not act_cli.wait_for_server(rospy.Duration.from_sec(1.)):
        rospy.logerr("Can't find action server")
        return
    arm = ArmInterface()
    kin = Kinematics(Kinematics.UR10_ID)
    # bin_man_moplan = PersonAvoidBinManMotionPlanner(arm, kin)
    bin_man_moplan = BinManagerMotionPlanner(arm, kin)
    bin_goal_planner = BinGoalPlanner(ar_man_sim)

    cur_dir = True
    while not rospy.is_shutdown():
        if cur_dir:
            cur_bins = right_slots.copy()
            cur_slots = left_slots.copy()
        else:
            cur_bins = left_slots.copy()
            cur_slots = right_slots.copy()
        rand_bin, rand_bin_loc = ar_man_sim.get_random_bin(cur_bins)
        rand_slot, rand_slot_loc = ar_man_sim.get_random_empty_slot(cur_slots)
        grasp_pose, place_pose = bin_goal_planner.generate_bin_location_goals(rand_bin, rand_slot)
        all_poses = [rand_bin_loc, rand_slot_loc, grasp_pose, place_pose]
        move_traj = bin_man_moplan.plan_bin_to_bin_traj(grasp_pose)
        # move_traj = bin_man_moplan.plan_home_traj()

        fjt = FollowJointTrajectoryGoal()
        fjt.trajectory = move_traj.to_trajectory_msg()
        fjt.trajectory.header.stamp = rospy.Time.now()
        fjt.trajectory.joint_names = arm.JOINT_NAMES

        act_cli.send_goal(fjt)
        rospy.loginfo("Starting trajectory")
        act_cli.wait_for_result()
        rospy.loginfo("Trajectory complete")
        
        bin_goal_planner.update_bin_location()
        bin_goal_planner.publish_bin_locs()
        cur_dir = not cur_dir

if __name__ == "__main__":
    main()
