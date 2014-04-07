#! /usr/bin/python

import numpy as np

import rospy
import smach
import smach_ros
import actionlib
from actionlib_msgs.msg import GoalStatus
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal

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
        return 'success'

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
    def __init__(self, bin_man_moplan, is_grasp):
        smach.State.__init__(self,
            outcomes=['success'],
            input_keys=['bin_move_goal'],
            output_keys=['bin_move_goal', 'joint_traj_goal'])
        self.bin_man_moplan = bin_man_moplan
        self.is_grasp = is_grasp

    def execute(self, userdata):
        if self.is_grasp:
            goal_pose = userdata.bin_move_goal['grasp_pose']
        else:
            goal_pose = userdata.bin_move_goal['place_pose']

        q_spline = self.bin_man_moplan.plan_bin_to_bin_traj(goal_pose)

        fjt = FollowJointTrajectoryGoal()
        fjt.trajectory = q_spline.to_trajectory_msg()
        fjt.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(0.05)
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

    if True:
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
        gripper.wait_for_connection()

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
        if is_grasp:
            move_type = 'GRASP'
        else:
            move_type = 'PLACE'
        sm_move_arm = smach.StateMachine(
                outcomes=['success', 'aborted', 'shutdown'],
                input_keys=['bin_move_goal'],
                output_keys=['bin_move_goal'])
        with sm_move_arm:
            smach.StateMachine.add(
                'GEN_MOPLAN_%s' % move_type, 
                SMGenerateMotionPlan(bin_man_moplan, is_grasp=is_grasp),
                transitions={'success' : 'JOINT_TRAJ_START_%s' % move_type})
            smach.StateMachine.add(
                'JOINT_TRAJ_START_%s' % move_type, 
                SMJointTrajActionStart(jnt_traj_act_cli),
                transitions={'success' : 'JOINT_TRAJ_WAIT_%s' % move_type,
                             'timeout' : 'aborted',
                             'shutdown' : 'shutdown'})
            smach.StateMachine.add(
                'JOINT_TRAJ_WAIT_%s' % move_type, 
                SMJointTrajActionWait(jnt_traj_act_cli),
                transitions={'success' : 'GRIPPER_%s' % move_type,
                             'preempted' : 'aborted',
                             'aborted' : 'aborted',
                             'shutdown' : 'shutdown'})
            smach.StateMachine.add(
                'GRIPPER_%s' % move_type, 
                SMGripperAction(gripper, is_grasp=is_grasp),
                transitions={'success' : 'success'})
        return sm_move_arm
    
    # generate full state machine
    sm = smach.StateMachine(outcomes=['aborted', 'shutdown'])
    sm.userdata.bin_move_goal = {}

    with sm:

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

    sis = smach_ros.IntrospectionServer('person_avoid_server', sm, '/SM_ROOT')
    sis.start()
    sm.execute()
    rospy.spin()
    sis.stop()

if __name__ == "__main__":
    main()
