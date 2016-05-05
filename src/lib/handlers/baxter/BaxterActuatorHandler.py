#!/usr/bin/env python

import argparse

# logger for ltlmop
import logging
ltlmop_logger = logging.getLogger('ltlmop_logger')

#import logging
#logging.basicConfig(level=ltlmop_logger.DEBUG)
#logger = logging.getLogger(__name__)

import rospy
import time

from dynamic_reconfigure.server import Server

from baxter_interface.cfg import (
    PositionJointTrajectoryActionServerConfig,
    VelocityJointTrajectoryActionServerConfig,
    PositionFFJointTrajectoryActionServerConfig,
)
from joint_trajectory_action.joint_trajectory_action import (
    JointTrajectoryActionServer,
)

from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)

import operator
import sys
import threading
import subprocess

from bisect import bisect
from copy import copy
from os import path

import actionlib

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
import baxter_interface

from baxter_interface import CHECK_VERSION

import lib.handlers.handlerTemplates as handlerTemplates

class BaxterActuatorHandler(handlerTemplates.ActuatorHandler):
    def __init__(self, executor, shared_data, limb, rate, mode):
        """
        Initalization of the two arms and grippers

        limb (string): joint trajectory action limb. You can enter both,left or right (default="both")
        rate (float): trajectory control rate in Hz. (default=100.0)
        mode (string): Control mode for trajectory execution. You can enter position_w_id,position or velocity (default="position_w_id")
        """

        self.baxterInitHandler = shared_data['BAXTER_INIT_HANDLER']

        self.dyn_cfg_srv = None # for traj action server
        self.jtas = [] # for track which limbs are used

        # initialize dictionaries
        self.traj = {} # for loading traj file
        self.play_traj_thread     = {} # execute play_traj asynchornously
        self.stop_traj_thread     = {} # execute stop_traj asynchornously
        self.play_traj_status_act = {} # track play_traj actuator status.
        self.baxterInitHandler.play_traj_status_sen = {} # track play_traj sensor status.
        self.trajLock = {} # for locking traj access
        rospy.on_shutdown(self._cleanup)

        ltlmop_logger.info("Initializing joint trajectory action server...")

        if mode == 'velocity':
            self.dyn_cfg_srv = Server(VelocityJointTrajectoryActionServerConfig,
                                 lambda config, level: config)
        elif mode == 'position':
            self.dyn_cfg_srv = Server(PositionJointTrajectoryActionServerConfig,
                                 lambda config, level: config)
        else:
            self.dyn_cfg_srv = Server(PositionFFJointTrajectoryActionServerConfig,
                                 lambda config, level: config)

        if limb == 'both':
            self.jtas.append(JointTrajectoryActionServer('right', self.dyn_cfg_srv, rate, mode))
            self.jtas.append(JointTrajectoryActionServer('left', self.dyn_cfg_srv, rate, mode))
        else:
            self.jtas.append(JointTrajectoryActionServer(limb, self.dyn_cfg_srv, rate, mode))

        self.pause = False

    def play_traj(self, fileName, actuatorName, loops, actuatorVal, initial=False):
        """
        This function plays a trajectory of baxter.

        fileName(string): name of the trajectory file, all located in the actions folder in handlers-baxter. (default="lib/handlers/baxter/actions/pickup.rec")
        actuatorName (string): name of this actuation
        loops (int): number of times that the action is played. 0 for infinite execution (default=1)
        """

        def commandToBaxter(loops, actuatorName, fileName):
            """
            send command to baxter. Specify # of loops
            """
            #try:
            # initialize trajectory
            self.trajLock[actuatorName].acquire()
            self.traj[actuatorName] = _Trajectory()
            self.traj[actuatorName].parse_file(path.expanduser(fileName))
            self.trajLock[actuatorName].release()

            #for safe interrupt handling
            rospy.on_shutdown(self.traj[actuatorName].stop)

            result = True
            loop_cnt = 1
            loopstr = loops
            if loops == 0:
                loops = float('inf')
                loopstr = "forever"
            while (result and loop_cnt <= loops and not rospy.is_shutdown() and self.traj[actuatorName]):
                ltlmop_logger.log(1,'is this running?')
                if not self.pause:
                    # execute trajectory
                    ltlmop_logger.debug('actuatorName:' + str(actuatorName))
                    ltlmop_logger.info("Playback loop %d of %d" % (loop_cnt, loopstr,))
                    ltlmop_logger.log(8,'do we start???')

                    self.trajLock[actuatorName].acquire()
                    if not self.pause and self.traj[actuatorName]:
                        ltlmop_logger.log(6, 'starting.... ')
                        self.traj[actuatorName].start()
                    self.trajLock[actuatorName].release()

                    ltlmop_logger.log(8,'do we block???')

                    # now check if action is done
                    if not self.pause:
                        self.trajLock[actuatorName].acquire()
                        if self.traj[actuatorName]:
                            result = self.traj[actuatorName].wait()
                        self.trajLock[actuatorName].release()

                        ltlmop_logger.debug('result:' + str(result))
                        ltlmop_logger.log(8,'checked result')

                        # increment loop only when the previous action is completed.
                        # if paused, result returns false, and we can resume with start() from previous point.
                        if result:
                            loop_cnt = loop_cnt + 1
                        # If paused before, this resets to the full action again
                        self.trajLock[actuatorName].acquire()
                        if loop_cnt <= loops and self.traj[actuatorName]:
                            self.traj[actuatorName] = _Trajectory()
                            self.traj[actuatorName].parse_file(path.expanduser(fileName))
                            ltlmop_logger.log('6', 'starting again.... ')
                            #self.traj[actuatorName].start()
                        self.trajLock[actuatorName].release()

                else:
                    time.sleep(0.5)
                    ltlmop_logger.log(6,'waiting to resume')

            ltlmop_logger.log(6,'Exited ' + actuatorName + 'execution')
            # ensure we quit because the action is completed. If pause, that means action is stopped
            if result and self.traj[actuatorName] and not self.traj[actuatorName].pause:
                ltlmop_logger.info("Exiting - File Playback Complete")
                ltlmop_logger.info("Action:" + actuatorName +" completed!")
                self.baxterInitHandler.play_traj_status_sen[actuatorName] = True

            # locking to ensure single access
            self.trajLock[actuatorName].acquire()
            if self.traj[actuatorName]:
                self.traj[actuatorName].stop()
                self.traj[actuatorName].stop
                del self.traj[actuatorName]
                self.traj[actuatorName] = None
            self.trajLock[actuatorName].release()

            #except:
            #    raise "CommandToBaxter: We ran into some errors!"

        def setSensorToFalse(time_delay, actuatorName):
            #time_delay in s
            # stop action if actuatorVal is false
            ltlmop_logger.warning('[ACT] In setSensorToFalse.')

            # locking to ensure single access

            if self.traj[actuatorName]:
                self.traj[actuatorName].pause = True
            self.trajLock[actuatorName].acquire()
            if self.traj[actuatorName]:
                ltlmop_logger.warning('[ACT] In setSensorToFalse.')
                self.traj[actuatorName].stop()
                self.traj[actuatorName].stop
                del self.traj[actuatorName]
                self.traj[actuatorName] = None
            self.trajLock[actuatorName].release()

            self.play_traj_status_act[actuatorName] = False
            ltlmop_logger.warning('[ACT] In setSensorToFalse.')
            time.sleep(0.2) #previously 0.5
            self.baxterInitHandler.play_traj_status_sen[actuatorName] = False
            ltlmop_logger.log(6,'[ACT] Sensor of action' + actuatorName + ' is false.')
            ltlmop_logger.warning('[ACT] In setSensorToFalse.')

        if initial:
            # initialize thread and status dict
            self.play_traj_thread[actuatorName] = None
            self.stop_traj_thread[actuatorName] = None
            self.play_traj_status_act[actuatorName] = False
            self.baxterInitHandler.play_traj_status_sen[actuatorName] = False
            self.traj[actuatorName] = None
            self.trajLock[actuatorName] = threading.Lock()
        else:
            if actuatorVal:
                # don't start action again if action has started
                if not self.play_traj_status_act[actuatorName]:
                    self.play_traj_thread[actuatorName] = threading.Thread(target=commandToBaxter, args=(loops, actuatorName, fileName))
                    self.play_traj_thread[actuatorName].daemon = True  # Daemonize thread. exit when exception occurs
                    self.play_traj_thread[actuatorName].start()
                    self.play_traj_status_act[actuatorName] = True

            else:
                #if self.play_traj_status_act[actuatorName]:
                ltlmop_logger.warning('stopping ACTION!!!!!')
                self.stop_traj_thread[actuatorName] = threading.Thread(target=setSensorToFalse, args=(0.1, actuatorName))
                self.stop_traj_thread[actuatorName].daemon = True  # Daemonize thread. exit when exception occurs
                self.stop_traj_thread[actuatorName].start()


    def _cleanup(self):
        ltlmop_logger.info("Cleaning jtas...")
        print "yo we are done!"
        for j in self.jtas:
            j.clean_shutdown()
        #ltlmop_logger.info("Running. Ctrl-c to quit")
        rospy.spin()

    def _pause(self):
        """
        temporarily pause all actions
        """
        self.pause = True
        for actuatorName, traj in self.traj.iteritems():
            if traj is not None:
                traj._pause()
                ltlmop_logger.debug('[ACT]'+ actuatorName + ' is paused.')

    def _resume(self):
        """
        resume all actions
        """
        # need to figure out what was running
        for actuatorName, traj in self.traj.iteritems():
            if traj is not None:
                traj._resume()
                ltlmop_logger.debug('[ACT]'+ actuatorName + ' is resumed.')
        self.pause = False

class _Trajectory(object):
    """
    This class comes from _joint_trajectory_file_playback
    """
    def __init__(self):
        self.setupConnection()

        self.now_from_start = 0.0
        ltlmop_logger.log(1,'initalizing')
        self.filename = "" #file of traj path
        self.pause = False
        self.aggregate_time = 0.0

        # Timing offset to prevent gripper playback before trajectory has started
        self._slow_move_offset = 0.0
        self._trajectory_start_offset = rospy.Duration(0.0)
        self._trajectory_actual_offset = rospy.Duration(0.0)

    def setupConnection(self):
       #create our action server clients
        self._left_client = actionlib.SimpleActionClient(
            'robot/limb/left/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )
        self._right_client = actionlib.SimpleActionClient(
            'robot/limb/right/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        #verify joint trajectory action servers are available
        l_server_up = self._left_client.wait_for_server(rospy.Duration(10.0))
        r_server_up = self._right_client.wait_for_server(rospy.Duration(10.0))
        if not l_server_up or not r_server_up:
            msg = ("Action server not available."
                   " Verify action server availability.")
            rospy.logerr(msg)
            rospy.signal_shutdown(msg)
            sys.exit(1)
        #create our goal request
        self._l_goal = FollowJointTrajectoryGoal()
        self._r_goal = FollowJointTrajectoryGoal()

        #limb interface - current angles needed for start move
        self._l_arm = baxter_interface.Limb('left')
        self._r_arm = baxter_interface.Limb('right')

        #gripper interface - for gripper command playback
        self._l_gripper = baxter_interface.Gripper('left', CHECK_VERSION)
        self._r_gripper = baxter_interface.Gripper('right', CHECK_VERSION)

        #flag to signify the arm trajectories have begun executing
        self._arm_trajectory_started = False
        #reentrant lock to prevent same-thread lockout
        self._lock = threading.RLock()

        # Verify Grippers Have No Errors and are Calibrated
        if self._l_gripper.error():
            self._l_gripper.reset()
        if self._r_gripper.error():
            self._r_gripper.reset()
        if (not self._l_gripper.calibrated() and
            self._l_gripper.type() != 'custom'):
            self._l_gripper.calibrate()
        if (not self._r_gripper.calibrated() and
            self._r_gripper.type() != 'custom'):
            self._r_gripper.calibrate()

        #gripper goal trajectories
        self._l_grip = FollowJointTrajectoryGoal()
        self._r_grip = FollowJointTrajectoryGoal()

        #param namespace
        self._param_ns = '/rsdk_joint_trajectory_action_server/'

        #gripper control rate
        self._gripper_rate = 20.0  # Hz

    def _pause(self):
        """
        pause current action
        """
        # here we stop the action
        self.stop()
        self.pause = True

    def _resume(self):
        """
        resume current action
        """
        ######################
        # initialize again ###
        ######################
        self.setupConnection()

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in joint_names:
                if 'left' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._l_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
                elif 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            #determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        ######################
        ### renew the traj ###
        ######################
        #open recorded file
        with open(self.filename, 'r') as f:
            lines = f.readlines()
        #read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        #parse joint names for the left and right limbs
        for name in joint_names:
            if 'left' == name[:-3]:
                self._l_goal.trajectory.joint_names.append(name)
            elif 'right' == name[:-3]:
                self._r_goal.trajectory.joint_names.append(name)

        ltlmop_logger.debug('self.now_from_start:' + str(self.now_from_start))
        ltlmop_logger.debug('self._slow_move_offset:' + str(self._slow_move_offset))
        init_point_added = False
        start_offset = 0.0

        # update time duration
        self.aggregate_time += self.now_from_start - self._slow_move_offset
        self.aggregate_time = max(0.0, self.aggregate_time)

        for idx, values in enumerate(lines[1:]):
            #clean each line of file
            cmd, values = self._clean_line(values, joint_names)

            #add a point for this set of commands with recorded time
            if values[0] >= self.aggregate_time:
                # first add current pos
                if not init_point_added:
                    ltlmop_logger.log(6,'init point adding now')
                    #now do a start offset?
                    cur_cmd = [self._l_arm.joint_angle(jnt) for jnt in self._l_goal.trajectory.joint_names]
                    self._add_point(cur_cmd, 'left', 0.0)
                    cur_cmd = [self._r_arm.joint_angle(jnt) for jnt in self._r_goal.trajectory.joint_names]
                    self._add_point(cur_cmd, 'right', 0.0)
                    start_offset = find_start_offset(cmd)
                    init_point_added = True
                    self._trajectory_start_offset = rospy.Duration(start_offset + values[0]- self.aggregate_time)

                #ltlmop_logger.debug('values[0]:'+ str(values[0]))
                #ltlmop_logger.log(8, values[0] + self._slow_move_offset - self.aggregate_time)
                timeStamp = values[0] - self.aggregate_time
                cur_cmd = [cmd[jnt] for jnt in self._l_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'left',timeStamp + start_offset)
                cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'right', timeStamp + start_offset)
                cur_cmd = [cmd['left_gripper']]
                self._add_point(cur_cmd, 'left_gripper', timeStamp + start_offset)
                cur_cmd = [cmd['right_gripper']]
                self._add_point(cur_cmd, 'right_gripper',timeStamp + start_offset)
        ltlmop_logger.log(8, 'total time:' + str(values[0] - self.aggregate_time + start_offset))
        ltlmop_logger.log(8,'aggregate_time:' + str(self.aggregate_time))
        self._slow_move_offset = start_offset

        # reset now from start
        #self.now_from_start = 0.0
        self.pause = False

        # start again
        #self.start()
        #ltlmop_logger.log(8, 'do we block?')

    def _execute_gripper_commands(self):
        start_time = rospy.get_time() - self._trajectory_actual_offset.to_sec()
        r_cmd = self._r_grip.trajectory.points
        l_cmd = self._l_grip.trajectory.points
        pnt_times = [pnt.time_from_start.to_sec() for pnt in r_cmd]
        end_time = pnt_times[-1]
        rate = rospy.Rate(self._gripper_rate)
        self.now_from_start = rospy.get_time() - start_time
        while(self.now_from_start < end_time + (1.0 / self._gripper_rate) and
              not rospy.is_shutdown() and not self.pause):
            idx = bisect(pnt_times, self.now_from_start) - 1
            if self._r_gripper.type() != 'custom':
                self._r_gripper.command_position(r_cmd[idx].positions[0])
            if self._l_gripper.type() != 'custom':
                self._l_gripper.command_position(l_cmd[idx].positions[0])
            rate.sleep()
            self.now_from_start = rospy.get_time() - start_time
            #ltlmop_logger.log(2,'self.now_from_start:' + str(self.now_from_start))


    def _clean_line(self, line, joint_names):
        """
        Cleans a single line of recorded joint positions

        @param line: the line described in a list to process
        @param joint_names: joint name keys

        @return command: returns dictionary {joint: value} of valid commands
        @return line: returns list of current line values stripped of commas
        """
        def try_float(x):
            try:
                return float(x)
            except ValueError:
                return None
        #convert the line of strings to a float or None
        line = [try_float(x) for x in line.rstrip().split(',')]
        #zip the values with the joint names
        combined = zip(joint_names[1:], line[1:])
        #take out any tuples that have a none value
        cleaned = [x for x in combined if x[1] is not None]
        #convert it to a dictionary with only valid commands
        command = dict(cleaned)
        return (command, line,)

    def _add_point(self, positions, side, time):
        """
        Appends trajectory with new point

        @param positions: joint positions
        @param side: limb to command point
        @param time: time from start for point in seconds
        """
        #creates a point in trajectory with time_from_start and positions
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        if side == 'left':
            self._l_goal.trajectory.points.append(point)
        elif side == 'right':
            self._r_goal.trajectory.points.append(point)
        elif side == 'left_gripper':
            self._l_grip.trajectory.points.append(point)
        elif side == 'right_gripper':
            self._r_grip.trajectory.points.append(point)

    def parse_file(self, filename):
        """
        Parses input file into FollowJointTrajectoryGoal format

        @param filename: input filename
        """
        #open recorded file
        self.filename = filename
        with open(filename, 'r') as f:
            lines = f.readlines()
        #read joint names specified in file
        joint_names = lines[0].rstrip().split(',')
        #parse joint names for the left and right limbs
        for name in joint_names:
            if 'left' == name[:-3]:
                self._l_goal.trajectory.joint_names.append(name)
            elif 'right' == name[:-3]:
                self._r_goal.trajectory.joint_names.append(name)

        def find_start_offset(pos):
            #create empty lists
            cur = []
            cmd = []
            dflt_vel = []
            vel_param = self._param_ns + "%s_default_velocity"
            #for all joints find our current and first commanded position
            #reading default velocities from the parameter server if specified
            for name in joint_names:
                if 'left' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._l_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
                elif 'right' == name[:-3]:
                    cmd.append(pos[name])
                    cur.append(self._r_arm.joint_angle(name))
                    prm = rospy.get_param(vel_param % name, 0.25)
                    dflt_vel.append(prm)
            diffs = map(operator.sub, cmd, cur)
            diffs = map(operator.abs, diffs)
            #determine the largest time offset necessary across all joints
            offset = max(map(operator.div, diffs, dflt_vel))
            return offset

        for idx, values in enumerate(lines[1:]):
            #clean each line of file
            cmd, values = self._clean_line(values, joint_names)
            #find allowable time offset for move to start position
            if idx == 0:
                # Set the initial position to be the current pose.
                # This ensures we move slowly to the starting point of the
                # trajectory from the current pose - The user may have moved
                # arm since recording
                cur_cmd = [self._l_arm.joint_angle(jnt) for jnt in self._l_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'left', 0.0)
                cur_cmd = [self._r_arm.joint_angle(jnt) for jnt in self._r_goal.trajectory.joint_names]
                self._add_point(cur_cmd, 'right', 0.0)
                start_offset = find_start_offset(cmd)
                # Gripper playback won't start until the starting movement's
                # duration has passed, and the actual trajectory playback begins
                self._slow_move_offset = start_offset
                self._trajectory_start_offset = rospy.Duration(start_offset + values[0])
            #add a point for this set of commands with recorded time
            cur_cmd = [cmd[jnt] for jnt in self._l_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'left', values[0] + start_offset)
            cur_cmd = [cmd[jnt] for jnt in self._r_goal.trajectory.joint_names]
            self._add_point(cur_cmd, 'right', values[0] + start_offset)
            cur_cmd = [cmd['left_gripper']]
            self._add_point(cur_cmd, 'left_gripper', values[0] + start_offset)
            cur_cmd = [cmd['right_gripper']]
            self._add_point(cur_cmd, 'right_gripper', values[0] + start_offset)

    def _feedback(self, data):
        # Test to see if the actual playback time has exceeded
        # the move-to-start-pose timing offset
        ltlmop_logger.log(4,'(>0)data.actual.time_from_start- self._trajectory_start_offset:' + str(data.actual.time_from_start-self._trajectory_start_offset))

        if (not self._get_trajectory_flag() and
              data.actual.time_from_start >= self._trajectory_start_offset):
            self._set_trajectory_flag(value=True)
            self._trajectory_actual_offset = data.actual.time_from_start

    def _set_trajectory_flag(self, value=False):
        with self._lock:
            # Assign a value to the flag
            self._arm_trajectory_started = value

    def _get_trajectory_flag(self):
        temp_flag = False
        with self._lock:
            # Copy to external variable
            temp_flag = self._arm_trajectory_started
        #ltlmop_logger.log(2,"temp_flag:" + str(temp_flag))
        return temp_flag

    def start(self):
        """
        Sends FollowJointTrajectoryAction request
        """
        self._left_client.send_goal(self._l_goal, feedback_cb=self._feedback)
        self._right_client.send_goal(self._r_goal, feedback_cb=self._feedback)
        # Syncronize playback by waiting for the trajectories to start
        while not rospy.is_shutdown() and not self._get_trajectory_flag() and not self.pause:
            rospy.sleep(0.05)
            ltlmop_logger.log(2,'in start')
        self._execute_gripper_commands()

    def stop(self):
        """
        Preempts trajectory execution by sending cancel goals
        """
        if (self._left_client.gh is not None and
            self._left_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._left_client.cancel_goal()

        if (self._right_client.gh is not None and
            self._right_client.get_state() == actionlib.GoalStatus.ACTIVE):
            self._right_client.cancel_goal()

        self.pause = True # stop start while loop
        #delay to allow for terminating handshake
        rospy.sleep(0.1)

    def wait(self):
        """
        Waits for and verifies trajectory execution result
        """
        #create a timeout for our trajectory execution
        #total time trajectory expected for trajectory execution plus a buffer
        last_time = self._r_goal.trajectory.points[-1].time_from_start.to_sec()
        time_buffer = rospy.get_param(self._param_ns + 'goal_time', 0.0) + 1.5
        timeout = rospy.Duration(self._slow_move_offset +
                                 last_time +
                                 time_buffer)

        l_finish = self._left_client.wait_for_result(timeout)
        r_finish = self._right_client.wait_for_result(timeout)
        l_result = (self._left_client.get_result().error_code == 0)
        r_result = (self._right_client.get_result().error_code == 0)

        # l_finish = self._left_client.wait_for_result(rospy.Duration(1.0))
        # r_finish = self._right_client.wait_for_result(rospy.Duration(1.0))
        # l_result = (self._left_client.get_result().error_code == 0)
        # r_result = (self._right_client.get_result().error_code == 0)

        # here we also pop the already execute points
        #ltlmop_logger.log(8,'l_finish:' + str(l_finish))

        #verify result
        if all([l_finish, r_finish, l_result, r_result]):
            return True
        else:
            msg = ("Trajectory action failed or did not finish before "
                   "timeout/interrupt.")
            rospy.logwarn(msg)
            return False

if __name__ == "__main__":

    a = BaxterActuatorHandler(None, None, 'both', 100, 'position_w_id')
    a.play_traj('lib/handlers/baxter/actions/pickup.rec', 'pickup', 1, 0, initial=True)
    a.play_traj('lib/handlers/baxter/actions/drop.rec', 'drop', 1, 0, initial=True)

    a.play_traj('lib/handlers/baxter/actions/pickup.rec', 'pickup', 1, 1, initial=False)
    rospy.signal_shutdown("Action completed.")
