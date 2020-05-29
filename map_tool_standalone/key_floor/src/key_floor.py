#!/usr/bin/env python

import sys, select
import tty, termios
import rospy
from std_msgs.msg import String

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('key_floor')
    pub = rospy.Publisher('floor', String, queue_size=1000)

    try:
        while(1):
            key = getKey()
            if (key == '\x03'):
                break
            elif key != '':
                msg = String()
                msg.data = key
                pub.publish(msg)
                print('msg pub')
    except:
        print('error')
    finally:
        msg.data = ''
        pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)