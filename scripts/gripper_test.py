#!/usr/bin/env python

import rospy
import baxter_interface
from baxter_interface import CHECK_VERSION

def main():

   rospy.init_node("gripper_test")
   rospy.sleep(0.25) # 1 sec   

   rs = baxter_interface.RobotEnable(CHECK_VERSION)
   init_state = rs.state().enabled

   left = baxter_interface.Gripper('left', CHECK_VERSION)
   rospy.sleep(0.25)
   block = False
   timeout = 5.0
   left.command_position(50.0, block, timeout)  

if __name__ == '__main__':
  main()
