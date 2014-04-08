#! /usr/bin/python

import numpy as np

import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from std_msgs.msg import Bool

from robotiq_c_model_control.robotiq_c_ctrl import RobotiqCGripper

from ur_py_utils.arm_iface import ArmInterface
from ur_py_utils.ur_controller_manager import URControllerManager
from ur_kin_py.kin import Kinematics

from bin_manager.ar_tag_manager_iface import ARTagManagerSimulator, setup_ar_tag_man_simulator
from bin_manager.bin_goal_planner import BinGoalPlanner
from bin_manager.bin_man_motion_plan import BinManagerMotionPlanner

class SimGripper(object):
    def __init__(self):
        pass

    def close(self, block):
        rospy.sleep(1.5)

    def goto(self, x, y, z, block):
        rospy.sleep(0.1)

class SMGripperAction(smach.State):
    def __init__(self, gripper, is_grasp):
        smach.State.__init__(self,
            outcomes=['success'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.gripper = gripper
        self.is_grasp = is_grasp
        # self.gripper = RobotiqCGripper()
        # self.gripper.wait_for_connection()

    def execute(self, userdata):
        if self.is_grasp:
            self.gripper.close(block=True)
        else:
            self.gripper.goto(0.042, 0., 0., block=False)
            rospy.sleep(0.5)
        return 'success'

class SMJointTrajActionWait(smach.State):
    def __init__(self, jnt_traj_act_cli):
        smach.State.__init__(self,
            outcomes=['success', 'preempted', 'aborted', 'shutdown'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.jnt_traj_act_cli = jnt_traj_act_cli

    def execute(self, ud):
        r = rospy.Rate(10)
        while True:
            if rospy.is_shutdown():
                self.jnt_traj_act_cli.cancel_all_goals()
                rospy.loginfo('SMJointTrajActionWait shutdown!')
                return 'shutdown'
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('SMJointTrajActionWait preempted!')
                return 'preempted'
            goal_state = self.jnt_traj_act_cli.get_state() 
            if goal_state != GoalStatus.PENDING and goal_state != GoalStatus.ACTIVE:
                if goal_state == GoalStatus.SUCCEEDED:
                    return 'success'
                if goal_state == GoalStatus.PREEMPTED:
                    return 'preempted'
                return 'aborted'
            r.sleep()

class SMStopMonitor(smach.State):
    def __init__(self, topic):
        smach.State.__init__(self,
            outcomes=['stop', 'preempted', 'shutdown'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.is_stop_requested = False
        self.stop_sub = rospy.Subscriber(topic, Bool, self.stop_cb)

    def stop_cb(self, stop_msg):
        self.is_stop_requested = stop_msg.data or self.is_stop_requested

    def execute(self, ud):
        self.is_stop_requested = False
        r = rospy.Rate(10)
        while True:
            if rospy.is_shutdown():
                rospy.loginfo('SMStopMonitor shutdown!')
                return 'shutdown'
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('SMStopMonitor preempted!')
                return 'preempted'
            if self.is_stop_requested:
                return 'stop'
            r.sleep()

class SMWaitForAllClear(smach.State):
    def __init__(self, stop_topic, all_clear_topic):
        smach.State.__init__(self,
            outcomes=['all_clear', 'preempted', 'shutdown'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.stop_sub = rospy.Subscriber(stop_topic, Bool, self.stop_cb)
        self.all_clear_sub = rospy.Subscriber(all_clear_topic, Bool, self.all_clear_cb)

    def stop_cb(self, stop_msg):
        self.cur_stop_state = stop_msg.data

    def all_clear_cb(self, all_clear_msg):
        self.cur_all_clear_state = all_clear_msg.data

    def execute(self, ud):
        self.cur_stop_state = True
        self.cur_all_clear_state = False
        r = rospy.Rate(10)
        while True:
            if rospy.is_shutdown():
                rospy.loginfo('SMWaitForAllClear shutdown!')
                return 'shutdown'
            if self.preempt_requested():
                self.service_preempt()
                rospy.loginfo('SMWaitForAllClear preempted!')
                return 'preempted'
            if not self.cur_stop_state and self.cur_all_clear_state:
                return 'all_clear'
            r.sleep()

class SMJointTrajActionStop(smach.State):
    def __init__(self, jnt_traj_act_cli):
        smach.State.__init__(self,
            outcomes=['success'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.jnt_traj_act_cli = jnt_traj_act_cli

    def execute(self, ud):
        self.jnt_traj_act_cli.cancel_all_goals()
        return 'success'

class SMBinMoveComplete(smach.State):
    def __init__(self, bin_goal_planner):
        smach.State.__init__(self,
            outcomes=['success'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.bin_goal_planner = bin_goal_planner

    def execute(self, ud):
        self.bin_goal_planner.update_bin_location()
        return 'success'

class SMGenerateRandomMoveGoals(smach.State):
    def __init__(self, bin_goal_planner, grasp_slots, place_slots):
        smach.State.__init__(self,
            outcomes=['success', 'aborted'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal'])
        self.bin_goal_planner = bin_goal_planner
        self.ar_tag_man = bin_goal_planner.ar_tag_man
        self.grasp_slots = grasp_slots
        self.place_slots = place_slots

    def execute(self, userdata):
        if len(userdata.bin_move_goal) == 0:
            cur_grasp_slots = self.grasp_slots
            cur_place_slots = self.place_slots
            prev_place_slot = None
        else:
            # get location of bin last placed
            prev_place_slot = userdata.bin_move_goal['place_slot_id']

            # only grasp from same side
            if prev_place_slot in self.grasp_slots:
                cur_grasp_slots = self.grasp_slots
                cur_place_slots = self.place_slots
            elif prev_place_slot in self.place_slots:
                cur_grasp_slots = self.place_slots
                cur_place_slots = self.grasp_slots
            else:
                rospy.logerr('Bin not in recognized slot, this should not happen')
                return 'aborted'

            # only grasp bins not just placed
            cur_grasp_slots = np.setdiff1d(cur_grasp_slots, [prev_place_slot])

        # print 'OLD', userdata.bin_move_goal
        # print 'cur_grasp_slots',  cur_grasp_slots
        # print 'prev_place_slot',  prev_place_slot
        # print 'cur_place_slots',  cur_place_slots
        # print 'self.grasp_slots', self.grasp_slots
        # print 'self.place_slots', self.place_slots
        rand_bin, rand_bin_loc = self.ar_tag_man.get_random_bin(cur_grasp_slots)
        if rand_bin is None:
            rospy.logerr('No other bins to move on same side, this should not happen')
            return 'aborted'
        rand_slot, rand_slot_loc = self.ar_tag_man.get_random_empty_slot(cur_place_slots)
        if rand_slot is None:
            rospy.logerr('No other slots to move to on opposite side, this should not happen')
            return 'aborted'
        grasp_pose, place_pose = self.bin_goal_planner.generate_bin_location_goals(
                                                                     rand_bin, rand_slot)
        bin_move_goal = {
            'grasp_bin_id' : rand_bin,
            'place_slot_id' : rand_slot,
            'grasp_pose' : grasp_pose,
            'place_pose' : place_pose
        }
        # print 'NEW', bin_move_goal
        userdata.bin_move_goal = bin_move_goal
        return 'success'

class SMGenerateMotionPlan(smach.State):
    # def __init__(self, bin_man_moplan, is_grasp):
    def __init__(self, bin_man_moplan, is_grasp, is_start_free, is_slow=False):
        smach.State.__init__(self,
            outcomes=['success'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal', 'joint_traj_goal'])
        self.bin_man_moplan = bin_man_moplan
        self.is_grasp = is_grasp
        self.is_start_free = is_start_free
        self.is_slow = is_slow

    def execute(self, userdata):
        if self.is_grasp:
            goal_pose = userdata.bin_move_goal['grasp_pose']
        else:
            goal_pose = userdata.bin_move_goal['place_pose']

        NORMAL_VEL = 0.47
        if self.is_slow:
            midpt_vel = NORMAL_VEL/3.
        else:
            midpt_vel = NORMAL_VEL

        now_time = rospy.Time.now()
        start_time = rospy.get_time()
        if self.is_start_free:
            q_spline = self.bin_man_moplan.plan_free_to_bin_traj(goal_pose, midpt_vel=midpt_vel)
        else:
            q_spline = self.bin_man_moplan.plan_bin_to_bin_traj(goal_pose)
        print rospy.get_time() - start_time

        fjt = FollowJointTrajectoryGoal()
        fjt.trajectory = q_spline.to_trajectory_msg()
        fjt.trajectory.header.stamp = now_time
        # fjt.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.05)
        fjt.trajectory.joint_names = self.bin_man_moplan.arm.JOINT_NAMES

        userdata.joint_traj_goal = fjt
        return 'success'

class SMJointTrajActionStart(smach.State):
    def __init__(self, jnt_traj_act_cli):
        smach.State.__init__(self,
            outcomes=['success', 'timeout', 'shutdown'],
            input_keys=['bin_move_goal', 'joint_traj_goal'],
            output_keys=['bin_move_goal'])
        self.jnt_traj_act_cli = jnt_traj_act_cli

    def execute(self, userdata):
        self.jnt_traj_act_cli.send_goal(userdata.joint_traj_goal)
        r = rospy.Rate(10)
        start_time = rospy.get_time()
        while True:
            if rospy.is_shutdown():
                self.jnt_traj_act_cli.cancel_all_goals()
                rospy.loginfo('SMJointTrajActionStart shutdown!')
                return 'shutdown'
            r.sleep()
            if self.jnt_traj_act_cli.get_state() == GoalStatus.ACTIVE:
                rospy.loginfo('Action successfully started')
                return 'success'
            if rospy.get_time() - start_time > 0.3:
                return 'timeout'



def main():
    rospy.init_node('sm_person_avoid')

    if False:
        # simulation setup
        ar_tag_man, bin_slots, bin_locs = setup_ar_tag_man_simulator()
        grasp_slots = np.array(range(0, 7))
        place_slots = np.array(range(7, 11))
        # print 'bin_slots', bin_slots.keys()
        # print 'bin_locs', bin_locs.keys()
        # return
        gripper = SimGripper()
    else:
        gripper = RobotiqCGripper()
        print 'Waiting for gripper...', 
        gripper.wait_for_connection()
        print 'found.'

    ################ setup utility classes #######################
    cman = URControllerManager()
    jnt_traj_act_cli = actionlib.SimpleActionClient(
            '/pva_trajectory_ctrl/follow_joint_trajectory', 
            FollowJointTrajectoryAction)
    if not jnt_traj_act_cli.wait_for_server(rospy.Duration.from_sec(1.)):
        rospy.logerr("Can't find action server")
        return
    arm = ArmInterface()

    kin = Kinematics(Kinematics.UR10_ID)
    bin_goal_planner = BinGoalPlanner(ar_tag_man)
    bin_man_moplan = BinManagerMotionPlanner(arm, kin)
    # bin_man_moplan = PersonAvoidBinManMotionPlanner(arm, kin)
    ###############################################################

    stop_topic = '/person_avoid/stop'
    all_clear_topic = '/person_avoid/all_clear'
    slow_topic = '/person_avoid/slow'

    # call this on shutdown:
    def shutdown_hook():
        jnt_traj_act_cli.cancel_all_goals()
    rospy.on_shutdown(shutdown_hook)

    # start pva joint trajectory controller
    cman.start_joint_controller('pva_trajectory_ctrl')

    ################ Move to home position #######################
    move_traj = bin_man_moplan.plan_home_traj()
    fjt = FollowJointTrajectoryGoal()
    fjt.trajectory = move_traj.to_trajectory_msg()
    fjt.trajectory.header.stamp = rospy.Time.now()
    fjt.trajectory.joint_names = arm.JOINT_NAMES

    jnt_traj_act_cli.send_goal(fjt)
    rospy.loginfo("Moving to home")
    jnt_traj_act_cli.wait_for_result()
    rospy.loginfo("Trajectory complete")
    ###############################################################

    def generate_move_sm(is_grasp):
        # generate a sub-state machine which does a grasp or a place
        sm_move_arm = smach.StateMachine(
                outcomes=['success', 'aborted', 'shutdown'],
                input_keys=['bin_move_goal'],
                output_keys=['bin_move_goal'])
        with sm_move_arm:
            smach.StateMachine.add(
                'GEN_MOPLAN', 
                SMGenerateMotionPlan(bin_man_moplan, is_grasp=is_grasp, is_start_free=False,
                                     is_slow=False),
                transitions={'success' : 'JOINT_TRAJ_START'})
            smach.StateMachine.add(
                'JOINT_TRAJ_START', 
                SMJointTrajActionStart(jnt_traj_act_cli),
                transitions={'success' : 'WAIT_MONITOR',
                             'timeout' : 'aborted',
                             'shutdown' : 'shutdown'})

            # build concurrence container for monitoring trajectory in motion
            wait_name = 'JOINT_TRAJ_WAIT'
            stop_mon_name = 'STOP_MONITOR'
            slow_mon_name = 'SLOW_MONITOR'

            def child_term_cb(outcome_map):
                if wait_name in outcome_map and outcome_map[wait_name]:
                    return True
                if stop_mon_name in outcome_map and outcome_map[stop_mon_name]:
                    return True
                if slow_mon_name in outcome_map and outcome_map[slow_mon_name]:
                    return True
                return False
            def out_cb(outcome_map):
                if outcome_map[stop_mon_name] == 'stop':
                    return 'stop'
                if outcome_map[slow_mon_name] == 'stop':
                    return 'slow'
                if outcome_map[wait_name] == 'success':
                    return 'success'
                return 'shutdown'
            sm_wait_monitor = smach.Concurrence(
                outcomes=['success', 'stop', 'slow', 'shutdown'],
                input_keys=['bin_move_goal'],
                output_keys=['bin_move_goal'],
                default_outcome='stop',
                child_termination_cb=child_term_cb,
                outcome_cb=out_cb)
            with sm_wait_monitor:
                smach.Concurrence.add(
                    wait_name, 
                    SMJointTrajActionWait(jnt_traj_act_cli))
                smach.Concurrence.add(
                    stop_mon_name, 
                    SMStopMonitor(stop_topic))
                smach.Concurrence.add(
                    slow_mon_name, 
                    SMStopMonitor(slow_topic))
            smach.StateMachine.add('WAIT_MONITOR', sm_wait_monitor,
                transitions={'success' : 'GRIPPER',
                             'stop' : 'JOINT_TRAJ_STOP',
                             'slow' : 'GEN_SLOW_REPLAN'})
            ###################################################################
            # stop trajectory and wait
            smach.StateMachine.add(
                'JOINT_TRAJ_STOP',
                SMJointTrajActionStop(jnt_traj_act_cli),
                transitions={'success' : 'WAIT_ALL_CLEAR'})

            smach.StateMachine.add(
                'WAIT_ALL_CLEAR',
                SMWaitForAllClear(stop_topic, all_clear_topic),
                transitions={'all_clear' : 'GEN_STOP_REPLAN',
                             'preempted' : 'aborted'})

            smach.StateMachine.add(
                'GEN_STOP_REPLAN', 
                SMGenerateMotionPlan(bin_man_moplan, is_grasp=is_grasp, is_start_free=True,
                                     is_slow=True),
                transitions={'success' : 'JOINT_TRAJ_START'})

            smach.StateMachine.add(
                'GEN_SLOW_REPLAN', 
                SMGenerateMotionPlan(bin_man_moplan, is_grasp=is_grasp, is_start_free=True,
                                     is_slow=True),
                transitions={'success' : 'JOINT_TRAJ_START'})

            # Grasp or release the current bin
            smach.StateMachine.add(
                'GRIPPER', 
                SMGripperAction(gripper, is_grasp=is_grasp),
                transitions={'success' : 'success'})
        return sm_move_arm
    
    sm_safety = smach.StateMachine(outcomes=['exit'])
    with sm_safety:

        # generate main state machine
        sm_main = smach.StateMachine(outcomes=['aborted', 'shutdown'])
        sm_main.userdata.bin_move_goal = {}

        with sm_main:

            smach.StateMachine.add(
                'GEN_RAND_MOVE_GOALS', 
                SMGenerateRandomMoveGoals(bin_goal_planner, grasp_slots, place_slots),
                transitions={'success' : 'GRASP',
                             'aborted' : 'aborted'})

            sm_grasp = generate_move_sm(True)
            smach.StateMachine.add(
                'GRASP', 
                sm_grasp,
                transitions={'success' : 'PLACE',
                             'aborted' : 'aborted',
                             'shutdown' : 'shutdown'})

            sm_place = generate_move_sm(False)
            smach.StateMachine.add(
                'PLACE', 
                sm_place,
                transitions={'success' : 'MOVE_COMPLETE',
                             'aborted' : 'aborted',
                             'shutdown' : 'shutdown'})

            smach.StateMachine.add(
                'MOVE_COMPLETE', 
                SMBinMoveComplete(bin_goal_planner),
                transitions={'success' : 'GEN_RAND_MOVE_GOALS'})

        smach.StateMachine.add(
            'MAIN_LOOP',
            sm_main,
            transitions={'aborted' : 'JOINT_TRAJ_STOP',
                         'shutdown' : 'JOINT_TRAJ_STOP'})
        smach.StateMachine.add(
            'JOINT_TRAJ_STOP',
            SMJointTrajActionStop(jnt_traj_act_cli),
            transitions={'success' : 'exit'})

    sis = smach_ros.IntrospectionServer('person_avoid_server', sm_safety, '/SM_ROOT')
    sis.start()
    sm_safety.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
