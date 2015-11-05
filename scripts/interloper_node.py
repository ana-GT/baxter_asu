#!/usr/bin/env python

from copy import copy
import rospy
import baxter_interface
import actionlib
from control_msgs.msg import (
FollowJointTrajectoryAction,
FollowJointTrajectoryGoal,
)

from trajectory_msgs.msg import (
JointTrajectoryPoint,
)

import interloper
import numpy as np
import ach

from baxter_interface import CHECK_VERSION


class Trajectory(object):
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=rospy.Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    def add_point(self, positions, time):
        point = JointTrajectoryPoint()
        point.positions = copy(positions)
        point.time_from_start = rospy.Duration(time)
        self._goal.trajectory.points.append(point)

    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    def stop(self):
        self._client.cancel_goal()

    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=rospy.Duration(timeout))

    def result(self):
        return self._client.get_result()

    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


class Updater:

    def __init__(self):
        """
        Initialization
        """
        self._limbs = ('left', 'right')
        self._arms = { 'left': baxter_interface.Limb('left'),
                       'right': baxter_interface.Limb('right') }
        

    def clean_shutdown(self):
        "Not sure what was here"

    def update_loop(self):
        
        control_rate = rospy.Rate(100)

	# Check if bimanual information is being sent
        #bimanual_chan = ach.Channel("bimanual_chan")
        #bimanual_chan.flush()
        #bimanual_msg = bytearray(10000)
        #[status,framesize]  = bimanual_chan.get( bimanual_msg, wait=False, last=False )
        #if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME :
        #    interloper.readGatekeeperMsg(bimanual_msg)
        

        while not rospy.is_shutdown():
            # Read values for left and for right arms
            pl = [ self._arms['left']._joint_angle[i] for i in self._arms['left']._joint_names['left'] ]
            vl = [ self._arms['left']._joint_velocity[i] for i in self._arms['left']._joint_names['left'] ]
            pr = [ self._arms['right']._joint_angle[i] for i in self._arms['right']._joint_names['right'] ]
            vr = [ self._arms['right']._joint_velocity[i] for i in self._arms['right']._joint_names['right'] ]


            # Send them with a sns_motor_state_msg
            interloper.sendArmStates( np.asarray(pl), np.asarray(vl), 'state-left' )
            interloper.sendArmStates( np.asarray(pr), np.asarray(vr), 'state-right' )
            
            # Check if a message is received
            

            # Send these as sns_msg_state 
            control_rate.sleep()

    def followTrajectoryLeft(self):
        traj = Trajectory('left')
        limb_interface = baxter_interface.Limb('left')
        current_angles = [ limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0 )
        p1 = [-0.11, -0.62, -1.15, 1.32, 0.80, 1.27, 2.39]
        traj.add_point(p1, 7.0)
        traj.add_point( [x*0.75 for x in p1], 9.0)
        traj.add_point( [x*1.25 for x in p1], 12.0)
        traj.start()
        traj.wait(15.0)
        print("Finished")


    def followTrajectoryRight(self):
        traj = Trajectory('right')
        limb_interface = baxter_interface.Limb('right')
        current_angles = [ limb_interface.joint_angle(joint) for joint in limb_interface.joint_names()]
        traj.add_point(current_angles, 0.0 )
        p1 = [0.11, -0.62, 1.15, 1.32, -0.80, 1.27, -2.39]
        traj.add_point(p1, 7.0)
        traj.add_point( [x*0.75 for x in p1], 9.0)
        traj.add_point( [x*1.25 for x in p1], 12.0)
        traj.start()
        traj.wait(15.0)
        print("Finished")

# Reading 
def main():

    rospy.init_node("interloper_node")
    updater = Updater()

    rospy.on_shutdown( updater.clean_shutdown )
    print("Starting updater...")
    #updater.update_loop()
    updater.followTrajectoryLeft()
    #updater.followTrajectoryRight()
    print( "Done! Finished my updating mission" )


# Main
if __name__ == '__main__':
    main()
