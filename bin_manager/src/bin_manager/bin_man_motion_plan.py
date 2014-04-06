#! /usr/bin/python

import numpy as np

import rospy

from trajectory_gen.traj_planner import TrajPlanner
from trajectory_gen.spline_traj_executor import SplineTraj

VEL_MULT = 4.7

class BinManagerMotionPlanner(object):
    def __init__(self, arm, kin):
        self.arm = arm
        self.kin = kin
        self.traj_plan = TrajPlanner(kin)
        self.grasp_lift = rospy.get_param("~grasp_lift", 0.18)
        self.pregrasp_vel = rospy.get_param("~pregrasp_vel", VEL_MULT*0.1)
        self.grasp_dur = rospy.get_param("~grasp_dur", 3.50/VEL_MULT)
        self.grasp_vel = rospy.get_param("~grasp_vel", 0.03*VEL_MULT)

        self.qd_max = [0.2]*6
        self.q_min = [-4.78, -2.4, 0.3, -3.8, -3.3, -2.*np.pi]
        self.q_max = [-1.2, -0.4, 2.7, -1.6, 0.3, 2.*np.pi]

    def plan_bin_to_bin_traj(self, goal_pose, mid_pts=[]):
        q_init = self.arm.get_q()
        x_init = self.kin.forward(q_init)

        q_knots = [q_init]
        t_knots = [0.]

        start_time = rospy.get_time() # time planning
        # move directly upward to clear bins and avoid bin collisions
        upward_pose = x_init.copy()
        upward_pose[2,3] += self.grasp_lift
        t_knots_new, q_knots_new = self.traj_plan.min_jerk_interp_ik(
                q_init=q_knots[-1], x_goal=upward_pose, dur=self.grasp_dur,
                vel_i=0., vel_f=self.grasp_vel,
                qd_max=self.qd_max, q_min=self.q_min, q_max=self.q_max)
        if q_knots_new is None:
            print 'move upward to pregrasp pose failed'
            print pregrasp_pose
            print q_knots
            return None
        print 'Planning time:', rospy.get_time() - start_time, len(t_knots_new)
        q_knots.extend(q_knots_new[1:])
        t_knots.extend(t_knots[-1] + t_knots_new[1:])

        # move to a waypoint above the goal
        above_goal_pose = goal_pose.copy()
        above_goal_pose[2,3] += self.grasp_lift
        prev_t = t_knots[-1]
        prev_q = q_knots[-1]

        # generate transversing trajectory
        t_midpt_knots, q_midpt_knots = self.create_midpt_traj(
                                            prev_t, prev_q, above_goal_pose)
        t_knots.extend(t_midpt_knots)
        q_knots.extend(q_midpt_knots)

        # move downward to goal pose
        start_time = rospy.get_time() # time planning
        t_knots_new, q_knots_new = self.traj_plan.min_jerk_interp_ik(
                q_init=q_knots[-1], x_goal=goal_pose, dur=self.grasp_dur,
                vel_i=self.grasp_vel, vel_f=0.,
                qd_max=self.qd_max, q_min=self.q_min, q_max=self.q_max)
        print 'Planning time:', rospy.get_time() - start_time, len(t_knots_new)
        q_knots.extend(q_knots_new[1:])
        t_knots.extend(t_knots[-1] + t_knots_new[1:])

        move_traj = SplineTraj.generate(t_knots, q_knots)
        return move_traj

    def create_midpt_traj(self, prev_t, prev_q, above_goal_pose):
        t_midpt_knots = []
        q_midpt_knots = []
        prev_pose = self.kin.forward(prev_q)

        # create midpts
        mid_pts = []

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

    def plan_home_traj(self):
        q_home = [-2.75203516454, -1.29936272152, 1.97292018645, 
                  -2.28456617769, -1.5054511996, -1.1]
        q_init = self.arm.get_q()
        start_time = rospy.get_time()
        t_knots, q_knots = self.traj_plan.min_jerk_interp_q_vel(q_init, q_home, self.pregrasp_vel/5.)
        print t_knots, q_knots
        print 'Planning time:', rospy.get_time() - start_time
        home_traj = SplineTraj.generate(t_knots, q_knots)
        #print t_knots, q_knots
        return home_traj

def main():
    from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
    import actionlib

    from ur_py_utils.arm_iface import ArmInterface
    from ur_py_utils.ur_controller_manager import URControllerManager
    from ar_tag_manager_iface import ARTagManagerSimulator, setup_ar_tag_man_simulator
    from ur_kin_py.kin import Kinematics

    from bin_goal_planner import BinGoalPlanner

    ar_man_sim, bin_slots, bin_locs = setup_ar_tag_man_simulator()
    right_slots = np.array(range(0, 7))
    left_slots = np.array(range(7, 11))

    rospy.init_node("test_traj_ctrl")
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
    bin_man_moplan = BinManagerMotionPlanner(arm, kin)
    bin_goal_planner = BinGoalPlanner(ar_man_sim)
    cur_bins = right_slots.copy()
    cur_slots = left_slots.copy()
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

if __name__ == "__main__":
    main()
