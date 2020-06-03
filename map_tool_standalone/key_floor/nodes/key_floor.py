#!/usr/bin/env python

import sys, select
import tty, termios
import rospy
from std_msgs.msg import String

class BreakoutException(Exception):
    pass

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
            msg = String()

            while len(msg.data) != 5:
                key = getKey()
                if (key == '\x03'):
                    raise BreakoutException
                else:
                    if len(msg.data) == 0:
                        if key != '':
                            msg.data = key
                    else:
                        if key != '':
                            msg.data += '_' + key
            pub.publish(msg)
            print(msg.data)
    except BreakoutException:
        pass
    finally:
        msg.data = ''
        pub.publish(msg)

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
