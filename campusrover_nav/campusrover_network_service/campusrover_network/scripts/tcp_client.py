#!/usr/bin/python
# license removed for brevity
import rospy
from sensor_msgs.msg import NavSatFix
import socket
import SocketServer
import json

PEV_SERVER_IP = '140.124.73.162'
PEV_SERVER_PORT = 1111
BUFFER_SIZE = 1024

class MyTCPHandler(SocketServer.BaseRequestHandler):
    """
    The request handler class for our server.
    It is instantiated once per connection to the server, and must
    override the handle() method to implement communication to the
    client.
    """
    def handle(self):
        # self.request is the TCP socket connected to the client
        global pub_location
        latlng = NavSatFix()
        self.data = self.request.recv(1024).strip()
        print self.data
        # data =  self.data.split(',')
        # if data[0] == 'L':
        #     latlng.header.stamp = rospy.Time.now()
        #     latlng.header.frame_id = "world"
        #     latlng.latitude = float(data[1])
        #     latlng.longitude = float(data[2])
        #     latlng.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        #     print "Location: " + data[0] + ' ' + data[1] + ", " + data[2]
        # elif data[0] == 'R':
        #     latlng.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        #     print "Request Release"
        # pub_location.publish(latlng)



if __name__ == "__main__":
    global pub_location
    rospy.init_node('talker', anonymous=True)
    print "Login PEV Server ..."
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        s.connect((PEV_SERVER_IP, PEV_SERVER_PORT))
    except socket.error as msg:
        s.close()
        s = None
        continue
    break
    if s is None:
        print 'faild to connect server!'
        sys.exit(1)
    s.send("{\"jsonrpc\":\"2.0\",\"method\":\"put\",\"params\":{\"key\":\"pev/1\"},\"id\":1}\n")
    while not rospy.is_shutdown():
        server_response_str = s.recv(BUFFER_SIZE)
        print server_response_str
        server_response_json = json.loads(server_response_str)
        if (server_response_json['id'] != 1):
            print "Error connect to server";
            sys.exit(0)
    s.close()
    # pub_location = rospy.Publisher('pev_gps_destination', NavSatFix, queue_size=10)
    # Create the server, binding to localhost on port 9999
    # print "PEV IP: " + pev_server_ip
    # server = SocketServer.TCPServer((pev_server_ip, 1112), MyTCPHandler)
    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    # server.serve_forever()
