#!/usr/bin/python
# license removed for brevity
import rospy
from sensor_msgs.msg import NavSatFix
import socket
import sys

PEV_SERVER_IP = '18.188.72.131'
PEV_SERVER_PORT = 1112
BUFFER_SIZE = 1024

def buffered_readLine(socket):
    line = ""
    while True:
        part = socket.recv(1)
        if part != "\n":
            line+=part
        elif part == "\n":
            break
    return line

def publishGps(my_data):
    global pub_location
    latlng = NavSatFix()
    data =  my_data.split(',')
    if data[0] == 'L':
        latlng.header.stamp = rospy.Time.now()
        latlng.header.frame_id = "world"
        latlng.latitude = float(data[1])
        latlng.longitude = float(data[2])
        latlng.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        print "Location: " + data[0] + ' ' + data[1] + ", " + data[2]
    elif data[0] == 'R':
        latlng.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        print "Request Release"
    pub_location.publish(latlng)

if __name__ == "__main__":
    global pub_location
    rospy.init_node('talker', anonymous=True)
    pub_location = rospy.Publisher('pev_gps_destination', NavSatFix, queue_size=10)
    print "Login PEV Server ..."
    while not rospy.is_shutdown():
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.connect((PEV_SERVER_IP, PEV_SERVER_PORT))
        except socket.error as msg:
            s.close()
            s = None
        if s is None:
            print 'faild to connect server!'
            sys.exit(1)
        data = buffered_readLine(s)
        publishGps(data)
        print data
    s.close()
