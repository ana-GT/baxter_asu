#!/usr/bin/python2

import os
import sys 
import argparse

import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import ( Image, )


# Define function
def send_image(path):
  """Send the image located at path to the head of Baxter """
  img = cv2.imread(path)
  msg = cv_bridge.CvBridge().cv2_to_imgmsg(img, encoding="bgr8")
  pub = rospy.Publisher('/robot/xdisplay', Image, latch=True, queue_size=1)
  pub.publish(msg)
  # SLeep to allow for the image to be published
  rospy.sleep(1)

# Main function
def main():
  """Display an image on Asu face"""
  epilog="""Max. screen resolution is 1024x600"""

  arg_fmt = argparse.RawDescriptionHelpFormatter
  parser = argparse.ArgumentParser( formatter_class=arg_fmt, description=main.__doc__,epilog=epilog )
  required = parser.add_argument_group('required arguments')
  required.add_argument('-f', '--file', metavar='PATH', required=True, help='Path to image file to set')

  parser.add_argument('-d','--delay', metavar='SEC', type=float,default=0.0, help='Time in seconds to wait before publishing image')

  args = parser.parse_args(rospy.myargv()[1:] )   


  rospy.init_node("an_3", anonymous=True)
  if not os.access( args.file, os.R_OK ):
    rospy.logerr("Cannot read input file at '%s' " % (args.file,))
    return 1
 
  # Wait for specified time, if delay given
  if args.delay > 0 :
    rospy.loginfo("Waiting for %s seconds before publishing image to face" % (args.delay,))
    rospy.sleep(args.delay)

  send_image( args.file )
  return 0


# Call from principal
if __name__ == '__main__':
  sys.exit(main())
