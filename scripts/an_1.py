#!/usr/bin/env python


"""
Example to read joints of arms, display them and move the arm forward
and backward when pressing two keys
"""
import argparse
import rospy
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

def map_keyboard():
  left_arm = baxter_interface.Limb('left')
  right_arm = baxter_interface.Limb('right')
  left_joints = left_arm.joint_names()
  right_joints = right_arm.joint_names()

  def set_j( _limb, _joint_name, _q ):
    j_cmd = {_joint_name:_q}
    _limb.set_joint_positions(j_cmd );

  def print_left():
    j_ang = left_arm.joint_angles()
    print "Left arm:"
    print "%f %f %f %f %f %f %f" % (j_ang['left_s0'], j_ang['left_s1'], j_ang['left_e0'], j_ang['left_e1'], j_ang['left_w0'], j_ang['left_w1'], j_ang['left_w2'] ) 
    
  def print_right():
    j_ang = right_arm.joint_angles()
    print "Right arm:"
    print "%f %f %f %f %f %f %f" % (j_ang['right_s0'], j_ang['right_s1'], j_ang['right_e0'], j_ang['right_e1'], j_ang['right_w0'], j_ang['right_w1'], j_ang['right_w2'] )



  done = False
  while not done and not rospy.is_shutdown():
    c = baxter_external_devices.getch()
    if c:
      if c in ['\x1b','\x03']:
        done=True
        rospy.signal.shutdown("Finished program")
      # 'p' to print
      elif c in ['p']:
        print_left()
        print_right() 

      # LEFT
      elif c in ['o']:
        j_ang = left_arm.joint_angles()
        dic_l = dict()
        for i in j_ang.keys():
          dic_l[i] = j_ang[i] + 0.1;
        left_arm.set_joint_positions(dic_l)
        print_left()
      elif c in ['l']:
        j_ang = left_arm.joint_angles()
        dic_l = dict()
        for i in j_ang.keys():
          dic_l[i] = j_ang[i] - 0.1
        left_arm.set_joint_positions(dic_l)
        print_left()

      # RIGHT
      elif c in ['w']:
        j_ang = right_arm.joint_angles()
        dic_r = dict()
        for i in j_ang.keys():
          dic_r[i] = j_ang[i] + 0.1;
        right_arm.set_joint_positions(dic_r)
        print_right()
      elif c in ['s']:
        j_ang = right_arm.joint_angles()
        dic_r = dict()
        for i in j_ang.keys():
          dic_r[i] = j_ang[i] - 0.1
        right_arm.set_joint_positions(dic_r)
        print_right()
       

# Main function
def main():
  """
  Main function
  """
  epilog= """ See help inside """
  arg_fmt = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser( formatter_class=arg_fmt,
                                    description=main.__doc__,
                                    epilog=epilog )
  parser.parse_args( rospy.myargv()[1:] )

  print("Initializing node...")
  rospy.init_node("an_1")
  print("Getting robot state")
  robot_state = baxter_interface.RobotEnable(CHECK_VERSION)
  init_state = robot_state.state().enabled

  # Shutdown routine (same as exit in C, I guess)
  def clean_shutdown():
    print("\n Exiting example ...")
    if not init_state:
      robot_state.disable()

  rospy.on_shutdown( clean_shutdown )
 
  print("Enabling robot..." )
  robot_state.enable()

  # Call function
  map_keyboard()
  print("Done!")

# Main
if __name__ == '__main__':
  main()
