#!/usr/bin/env python

"""
Send arm states to ach_channels
Read gatekeeper messages to control the arms
Implement a client node to do the above thing
"""

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
JointTrajectory
)

import interloper
import numpy as np
import ach
import rospy.rostime
from rospy.rostime import Duration

from baxter_interface import CHECK_VERSION

# *************************
# @class Trajectory
# *************************
class Trajectory(object):

    """
    Initialization function
    Connect with server
    """
    def __init__(self, limb):
        ns = 'robot/limb/' + limb + '/'
        self._client = actionlib.SimpleActionClient(
            ns + "follow_joint_trajectory",
            FollowJointTrajectoryAction,
        )
        self._goal = FollowJointTrajectoryGoal()
        self._goal_time_tolerance = rospy.Time(0.1)
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        server_up = self._client.wait_for_server(timeout=Duration(10.0))
        if not server_up:
            rospy.logerr("Timed out waiting for Joint Trajectory"
                         " Action Server to connect. Start the action server"
                         " before running example.")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)
        self.clear(limb)

    """
    Add trajectory points to be executed
    in the trajectory following
    """
    def add_point(self, _joints, _time):
        point = JointTrajectoryPoint()
        point.positions = copy(_joints)
        point.time_from_start = Duration(_time)
        self._goal.trajectory.points.append(point)

    """
    Send the trajectory to follow and the service starts rolling
    """
    def start(self):
        self._goal.trajectory.header.stamp = rospy.Time.now()
        self._client.send_goal(self._goal)

    """
    Cancel the trajectory following
    """
    def stop(self):
        self._client.cancel_goal()

    """
    Wait till the trajectory is finished
    the timeout should be dictated by the trajectory itself
    """
    def wait(self, timeout=15.0):
        self._client.wait_for_result(timeout=Duration(timeout))

    """
    Get result of trajectory following
    """
    def result(self):
        return self._client.get_result()

    """
    Clear trajectory
    """
    def clear(self, limb):
        self._goal = FollowJointTrajectoryGoal()
        self._goal.goal_time_tolerance = self._goal_time_tolerance
        self._goal.trajectory.joint_names = [limb + '_' + joint for joint in \
            ['s0', 's1', 'e0', 'e1', 'w0', 'w1', 'w2']]


"""
Runs a loop that sends robot arm states
and waits for gatekeeper messages
"""
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

    """
    Run forever waiting for inputs or outputting states
    """
    def update_loop(self):
        
        control_rate = rospy.Rate(100)
        
        bimanual_chan = ach.Channel("bimanual_chan")
        bimanual_chan.flush()
        bimanual_msg = bytearray(100000)
        jt_msg_left = JointTrajectory()
	jt_msg_right = JointTrajectory()
        
        while not rospy.is_shutdown():

            # Grab arm states and send them to robot
            pl = [ self._arms['left']._joint_angle[i] for i in self._arms['left']._joint_names['left'] ]
            vl = [ self._arms['left']._joint_velocity[i] for i in self._arms['left']._joint_names['left'] ]
            pr = [ self._arms['right']._joint_angle[i] for i in self._arms['right']._joint_names['right'] ]
            vr = [ self._arms['right']._joint_velocity[i] for i in self._arms['right']._joint_names['right'] ]

            interloper.sendArmStates( np.asarray(pl), np.asarray(vl), 'state-left' )
            interloper.sendArmStates( np.asarray(pr), np.asarray(vr), 'state-right' )
            
            # Check if bimanual information was received
            [status,framesize]  = bimanual_chan.get( bimanual_msg, wait=False, last=True )
            if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME :
                (jt_msg_left_ser, mode, jt_msg_right_ser) = interloper.readGatekeeperMsg(bimanual_msg) 
                print "Mode python: ", mode
                print "Size left: ", len(jt_msg_left_ser), " and right: ", len(jt_msg_right_ser)
	        
                if len(jt_msg_left_ser) > 0 :
                    print "Follow trajectory left"
                    jt_msg_left.deserialize( jt_msg_left_ser )
                    self.followTrajectory( jt_msg_left, 'left' )
                if len(jt_msg_right_ser) > 0 :
                    print "Follow trajectory right"
                    jt_msg_right.deserialize( jt_msg_right_ser )
                    self.followTrajectory( jt_msg_right, 'right' )                
                
                # Clean after yourself
                bimanual_chan.flush()

            # Send these as sns_msg_state 
            control_rate.sleep()

    """
    FollowTrajectory
    """
    def followTrajectory(self, _msg, _limb):

        if len( _msg.points ) == 0:
  	    print "Trajectory of limb ", _limb, " has size 0, so not executing"
            return

        traj = Trajectory(_limb)
        limb_interface = self._arms[_limb]
        print "Trajectory ", _limb," has ", len(_msg.points)
        for p in _msg.points:
            traj.add_point( p.positions, (p.time_from_start).to_sec() )

        traj.start()
        traj.wait(15.0)
        print "Finished following trajectory for arm: ", _limb
        interloper.sendGatekeeperMsg_arm( _limb, 1, "gatekeeper_msg_chan" )



# Main function
def main():

    rospy.init_node("interloper_node")
    updater = Updater()

    rospy.on_shutdown( updater.clean_shutdown )
    print("Starting updater...")
    updater.update_loop()
    print( "Done! Finished my updating mission" )


# Main
if __name__ == '__main__':
    main()
