#!/usr/bin/env python

import rospy
import numpy as np
import interloper
import ach
from types import *

from trajectory_msgs.msg import (
JointTrajectory, JointTrajectoryPoint
)


# Class Updater
class Updater:
    
    def __init__(self):
        """Initialization"""
        
    def clean_shutdown(self):
        """Not sure what to put here"""

    def update_loop(self):
        control_rate = rospy.Rate(100)
        bimanual_chan = ach.Channel("bimanual_chan")
        bimanual_chan.flush()
        bimanual_msg = bytearray(10000)

        while not rospy.is_shutdown():
            [status,framesize]  = bimanual_chan.get( bimanual_msg, wait=False, last=False )
            if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME :
                minnie = JointTrajectory()
                barf = interloper.readGatekeeperMsg(bimanual_msg)
                minnie.deserialize(barf)
                print "Min points: ", len(minnie.points)
                print "Point 1:", minnie.points[0]
            control_rate.sleep()


def main():
    rospy.init_node("test_interloper_gk")
    updater = Updater()
    rospy.on_shutdown(updater.clean_shutdown )
    print "Starting updater"
    updater.update_loop()
    print "Done with updating"


# Main call
if __name__ == '__main__':
    main()
