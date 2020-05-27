#!/usr/bin/python
# license removed for brevity
import rospy
from sensor_msgs.msg import NavSatFix
import socket
import SocketServer
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
        data =  self.data.split(',')
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
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect(('192.168.3.1', 0))               # Connet to your gateway
    my_ip = s.getsockname()[0]
    print my_ip
    HOST, PORT = my_ip, 9999		# Get current IP
    rospy.init_node('talker', anonymous=True)
    pub_location = rospy.Publisher('pev_gps_destination', NavSatFix, queue_size=10)
    # Create the server, binding to localhost on port 9999
    server = SocketServer.TCPServer((HOST, PORT), MyTCPHandler)
    # Activate the server; this will keep running until you
    # interrupt the program with Ctrl-C
    server.serve_forever()
