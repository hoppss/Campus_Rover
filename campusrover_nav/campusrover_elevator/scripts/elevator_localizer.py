#!/usr/bin/env python

import rospy
import roslaunch
import threading
import time
from std_msgs.msg import String
from campusrover_msgs.msg import ElevatorControlStatus

def startLaunch():
  global first_time_

  rospy.loginfo( "Start Launch")
  if(first_time_ == False):
    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, ["/home/luke/campusrover_ws/src/Campus_Rover/campusrover_slam/campusrover_odom/launch/campusrover_2d_localization.launch"])
    launch.start()

    rospy.loginfo("started")
    first_time_ = True

def ControlStatusCallback(data):
  control_status_ = data.control_status

  global first_time_, thread_1_
  rospy.loginfo( "I heard %s", control_status_)
  first_time_ = False


  

def listener():

  rospy.init_node('elevator_localizer', anonymous=True)
  rospy.Subscriber("control_status", ElevatorControlStatus, ControlStatusCallback)
  rospy.spin()


if __name__ == '__main__':
  global first_time_
  global control_status_
  global thread_1_, thread_2_

  first_time_ = True
  control_status_ = 0

  thread_1_ = threading.Thread(target=startLaunch)
  thread_1_.start()
  
  # thread_2_ = threading.Thread(target=listener)
  # thread_2_.start()
  # thread_2_.join()

  listener()

  
  

