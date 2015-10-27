#!/usr/bin/env python

import rospy
import baxter_interface

def main():
  print("Init node example")
  rospy.init_node("an_2")
  
  # Get limb
  limb = baxter_interface.Limb('right')
  # Get the right limb's current joint angles
  angles = limb.joint_angles()

  # Print the angles
  print angles

  # Reassign the new joint angles
  val = 0.0 
  angles['right_s0'] = val
  angles['right_s1'] = val
  angles['right_e0'] = val
  angles['right_e1'] = val
  angles['right_w0'] = val
  angles['right_w1'] = val
  angles['right_w2'] = val

  # Print command
  print angles

  # Move the right arm to those joint angles
  limb.move_to_joint_positions(angles)

  # Wave - Position 1
  wave_1 = {'right_s0': -0.459, 'right_s1': -0.202, 'right_e0':1.807, 'right_e1':1.714, 'right_w0':-0.906, 'right_w1':-1.545, 'right_w2':-0.276}


  # Wave - Position 2
  wave_2 = {'right_s0': -0.395, 'right_s1': -0.202, 'right_e0':1.831, 'right_e1':1.981, 'right_w0':-1.979, 'right_w1':-1.1, 'right_w2':-0.448}

  # Move (wave) 3 times
  for _mode in range(3):
    limb.move_to_joint_positions(wave_1)
    limb.move_to_joint_positions(wave_2)

# Main
if __name__ == '__main__':
  main()


