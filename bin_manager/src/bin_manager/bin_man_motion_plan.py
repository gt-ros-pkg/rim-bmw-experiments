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
        self.q_min = [-2.*np.pi, -2.4, 0.0, -3.1, -3.3, -2.*np.pi]
        self.q_max = [2.*np.pi, 0.0, 2.8, -1.0, 0.3, 2.*np.pi]
        self.q_home = [0.7678, -1.1937,  1.5725, -1.9750, -1.573, 0.0]

    def plan_bin_to_bin_traj(self, goal_pose, front_requested, is_grasp, mid_pts=[], midpt_vel=None):
        q_init = self.arm.get_q()
        qd_init = self.arm.get_qd()
        x_init = self.kin.forward(q_init)

        q_knots = [q_init]
        t_knots = [0.]

        q_min = np.array(self.q_min).copy()
        q_max = np.array(self.q_max).copy()

        start_time = rospy.get_time() # time planning
        # move directly upward to clear bins and avoid bin collisions
        upward_pose = x_init.copy()
        upward_pose[2,3] += self.grasp_lift
        t_knots_new, q_knots_new = self.traj_plan.min_jerk_interp_ik(
                q_init=q_knots[-1], x_goal=upward_pose, dur=self.grasp_dur,
                vel_i=0., vel_f=self.grasp_vel,
                qd_max=self.qd_max, q_min=q_min, q_max=q_max)
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
        # t_midpt_knots, q_midpt_knots = self.create_midpt_traj_new(
        t_midpt_knots, q_midpt_knots = self.create_midpt_traj(
                                            prev_t, prev_q, above_goal_pose, 
                                            is_grasp, front_requested, qd_init, midpt_vel)
        # print q_knots, 'YOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO'
        # print q_midpt_knots, 'XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX'
        t_knots.extend(t_midpt_knots)
        q_knots.extend(q_midpt_knots)

        # move downward to goal pose
        start_time = rospy.get_time() # time planning
        t_knots_new, q_knots_new = self.traj_plan.min_jerk_interp_ik(
                q_init=q_knots[-1], x_goal=goal_pose, dur=self.grasp_dur,
                vel_i=self.grasp_vel, vel_f=0.,
                qd_max=self.qd_max, q_min=q_min, q_max=q_max)
        print 'Planning time:', rospy.get_time() - start_time, len(t_knots_new)
        q_knots.extend(q_knots_new[1:])
        t_knots.extend(t_knots[-1] + t_knots_new[1:])

        move_traj = SplineTraj.generate(t_knots, q_knots)
        return move_traj

    def plan_free_to_bin_traj(self, goal_pose, front_requested, is_grasp, mid_pts=[], midpt_vel=None):
        if midpt_vel is None:
            midpt_vel = self.pregrasp_vel
        start_time = rospy.get_time()
        q_init = self.arm.get_q()
        qd_init = self.arm.get_qd()
        x_init = self.kin.forward(q_init)
        print '------------------------XXXX--------------', rospy.get_time() - start_time

        q_knots = [q_init]
        t_knots = [0.]

        q_min = np.array(self.q_min).copy()
        q_max = np.array(self.q_max).copy()

        # move to a waypoint above the goal
        above_goal_pose = goal_pose.copy()
        above_goal_pose[2,3] += self.grasp_lift
        prev_t = t_knots[-1]
        prev_q = q_knots[-1]

        # generate transversing trajectory
        # t_midpt_knots, q_midpt_knots = self.create_midpt_traj_new(
        t_midpt_knots, q_midpt_knots = self.create_midpt_traj(
                                            prev_t, prev_q, above_goal_pose, 
                                            is_grasp, front_requested, qd_init, midpt_vel)
        t_knots.extend(t_midpt_knots)
        q_knots.extend(q_midpt_knots)

        # move downward to goal pose
        start_time = rospy.get_time() # time planning
        t_knots_new, q_knots_new = self.traj_plan.min_jerk_interp_ik(
                q_init=q_knots[-1], x_goal=goal_pose, dur=self.grasp_dur,
                vel_i=self.grasp_vel, vel_f=0.,
                qd_max=self.qd_max, q_min=q_min, q_max=q_max)
        print 'Planning time:', rospy.get_time() - start_time, len(t_knots_new)
        q_knots.extend(q_knots_new[1:])
        t_knots.extend(t_knots[-1] + t_knots_new[1:])

        move_traj = SplineTraj.generate(t_knots, q_knots, qd_i=qd_init)
        return move_traj

    def get_q1_goal(self, prev_q, new_pose, front_requested):

        new_pose_ang = np.arctan2(new_pose[1,3], new_pose[0,3])
        goal_left = new_pose_ang > 0.
        print 'goal_left', goal_left, new_pose_ang, new_pose[:2,3]

        q1 = prev_q[0]
        print 'q1', q1
        left_min = np.pi/2
        left_max = np.pi
        right_min = 3*np.pi/2
        right_max = 2*np.pi
        if goal_left:
            if q1 <= -np.pi/2:
                return left_min-2*np.pi, left_max-2*np.pi
            elif q1 >= 0.:
                return left_min, left_max
            elif front_requested:
                return left_min, left_max
            else:
                return left_min-2*np.pi, left_max-2*np.pi
        else:
            if q1 <= np.pi/2:
                return right_min-2*np.pi, right_max-2*np.pi
            elif q1 >= np.pi:
                return right_min, right_max
            elif front_requested:
                return right_min-2*np.pi, right_max-2*np.pi
            else:
                return right_min, right_max


        # left_min = np.pi/2
        # left_max = np.pi
        # right_min = -np.pi/2
        # right_max = 0.
        # q1 = prev_q[0]
        # if q1 < 0.:
        #     start_neg = True
        # else:
        #     start_neg = False
        # if new_pose_ang > left_min and new_pose_ang < left_max:
        #     end_left = True
        # elif new_pose_ang > right_min and new_pose_ang < right_max:
        #     end_left = False
        # else:
        #     rospy.logerr('end pose not in range..')
        #     return None, None

        # is_backwards = not front_requested
        # if start_neg and not end_left:
        #     # gotta go backwards
        #     is_backwards = True
        # if not start_neg and end_left:
        #     # gotta go backwards
        #     is_backwards = True

        # if is_backwards:
        #     if start_neg:
        #         add_ang = 2*np.pi
        #     else:
        #         add_ang = 0
        #     if end_left:
        #         return right_min - np.pi + add_ang, right_max - np.pi + add_ang
        #     else:
        #         return left_min - np.pi + add_ang, left_max - np.pi + add_ang
        # else:
        #     # is front traj
        #     if end_left:
        #         return right_min + np.pi, right_max + np.pi
        #     else:
        #         return left_min - np.pi, left_max - np.pi

    def get_q_bounds(self, prev_q, new_pose, front_requested):
        q1_min, q1_max = self.get_q1_goal(prev_q, new_pose, front_requested)
        q_min = np.array(self.q_min).copy()
        q_max = np.array(self.q_max).copy()
        q_min[0] = q1_min
        q_max[0] = q1_max
        return q_min, q_max

    def create_midpt_traj(self, prev_t, prev_q, above_goal_pose, 
                          is_grasp, front_requested, qd_init=[0.]*6, midpt_vel=None):
        if midpt_vel is None:
            midpt_vel = self.pregrasp_vel
        t_midpt_knots = []
        q_midpt_knots = []
        prev_pose = self.kin.forward(prev_q)

        # create midpts
        mid_pts = []

        for mid_pose in mid_pts + [above_goal_pose]: 

            if is_grasp:
                print 'IS GRASP'
                q_min = np.array(self.q_min).copy()
                q_max = np.array(self.q_max).copy()
            else:
                print 'IS PLACE'
                q_min, q_max = self.get_q_bounds(prev_q, above_goal_pose, front_requested)
            q_mid_pose = self.kin.inverse(mid_pose, prev_q,
                                          q_min=q_min, q_max=q_max)
            print 'prev_pose', prev_pose
            print 'mid_pose', mid_pose
            print 'prev_q', prev_q
            print 'q_min', q_min
            print 'q_mid_pose', q_mid_pose
            print 'q_max', q_max
            if q_mid_pose is None:
                rospy.logerr('Failed to find midpoint IK')
                print 'mid_pose', mid_pose
                print 'prev_q', prev_q
                print 'q_min', q_min
                print 'ALL SOLUTIONS'
                print self.kin.inverse_all(mid_pose, prev_q)
                return None
            q_midpt_knots.append(q_mid_pose)
            # use a heuristic time based on the velocity to select a knot time
            dist = np.linalg.norm(mid_pose[:3,3]-prev_pose[:3,3])
            cur_t_delta = max(dist / midpt_vel, 0.3)
            t_midpt_knots.append(prev_t + cur_t_delta)
            prev_t += cur_t_delta
            prev_q = q_mid_pose
            prev_pose = mid_pose
        return t_midpt_knots, q_midpt_knots

    def create_midpt_traj_new(self, prev_t, prev_q, above_goal_pose, qd_init=[0.]*6, midpt_vel=None):
        if midpt_vel is None:
            midpt_vel = self.pregrasp_vel
            is_slow = False
        else:
            is_slow = True
        q_goal_pose = self.kin.inverse(above_goal_pose, prev_q,
                                       q_min=q_min, q_max=q_max)
        prev_pose = self.kin.forward(prev_q)
        dist = np.linalg.norm(above_goal_pose[:3,3]-prev_pose[:3,3])

        if not is_slow:

            # use a heuristic time based on the velocity to select a knot time
            cur_t_delta = max(dist / midpt_vel, 0.3)

            num_t_knots = round(cur_t_delta / 0.08)
            t_knots = np.linspace(0., cur_t_delta, num_t_knots)
            q_knots, qd_knots, qdd_knots = self.traj_plan.min_jerk_interp_q_knots(
                prev_q, q_goal_pose, t_knots, qd_i=qd_init, qd_f=[0.]*6, qdd_i=[0.]*6, qdd_f=[0.]*6)

            q_midpt_knots = q_knots
            t_midpt_knots = (t_knots + prev_t).tolist()
            return t_midpt_knots[1:-3], q_midpt_knots[1:-3]
        else:
            # use a heuristic time based on the velocity to select a knot time
            t_delta = 0.7
            q_i = np.array(prev_q)
            qd_i = np.array(qd_init)
            qd_f = qd_i*0.3
            q_f = q_i + (qd_f - qd_i) * t_delta
            num_t_knots = round(t_delta / 0.08)
            t_knots1 = np.linspace(0., t_delta, num_t_knots)

            q_knots1, qd_knots1, qdd_knots1 = self.traj_plan.min_jerk_interp_q_knots(
                q_i, q_f, t_knots1, qd_i=qd_i, qd_f=qd_f, qdd_i=[0.]*6, qdd_f=[0.]*6)

            prev_pose = self.kin.forward(q_f)
            dist = np.linalg.norm(above_goal_pose[:3,3]-prev_pose[:3,3])
            t_delta = max(dist / midpt_vel, 0.3)
            q_i = q_f
            qd_i = qd_f
            qd_f = [0.]*6
            q_f = q_goal_pose
            num_t_knots = round(t_delta / 0.08)
            t_knots2 = np.linspace(0., t_delta, num_t_knots)

            q_knots2, qd_knots2, qdd_knots2 = self.traj_plan.min_jerk_interp_q_knots(
                q_i, q_f, t_knots2, qd_i=qd_i, qd_f=qd_f, qdd_i=[0.]*6, qdd_f=[0.]*6)

            q_midpt_knots = q_knots1[1:].tolist() + q_knots2[1:].tolist()
            t_knots1 += prev_t
            t_knots2 += t_knots1[-1]
            t_midpt_knots = t_knots1[1:].tolist() + t_knots2[1:].tolist()
            return t_midpt_knots[:-3], q_midpt_knots[:-3]

    def plan_home_traj(self):
        q_home = self.q_home
        q_init = self.arm.get_q()
        start_time = rospy.get_time()
        t_knots, q_knots = self.traj_plan.min_jerk_interp_q_vel(q_init, q_home, self.pregrasp_vel/3.)
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
