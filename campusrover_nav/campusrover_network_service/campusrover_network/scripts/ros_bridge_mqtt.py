#!/usr/bin/python
import paho.mqtt.client as mqtt
import rospy
import json
import signal
import yaml
import socket
import fcntl
import struct
from std_msgs.msg import String, UInt8,Bool,ColorRGBA
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
from autolabor_msgs.srv import *

gps_data_save = []

class ServiceMode():
    Pending   = 0
    Auto      = 1
    Manual    = 2
    Following = 3

class DriveMode():
    lock               = 1
    centering_steering = 2
    set_steering       = 3
    release            = 4
    manual             = 5
    auto_mode          = 6

def callDriveService(mode, message):
    rospy.wait_for_service('drive_status')
    try:
        drive_status = rospy.ServiceProxy('drive_status', DriveStatus)
        drive_status(mode, message) 
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
    
def getHwAddr(ifname):
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        info = fcntl.ioctl(s.fileno(), 0x8927,
                           struct.pack('256s', ifname[:15]))
        get_mac = ''.join(['%02x:' % ord(char) for char in info[18:24]])[:-1]
        return_mac = get_mac
    except:
        # print "Caught exception socket.error : %s" % exc
        return_mac = "00:00:00:00:00:00"
    return return_mac

def mac_addr(ifname):
    get_mac = getHwAddr(ifname)
    get_mac = str.upper(get_mac.replace(":", ""))
    return get_mac

get_local_mac = mac_addr("wlp2s0")

def on_disconnect(client, userdata, rc=0):
    client.loop_stop()

def on_connect(mq, userdata, rc, _):
    print("Connected with result code "+str(rc))
    client.subscribe("pev/#")

def on_message(mq, userdata, msg):
    print("Topic: %s" % msg.topic)
    get_data = json.loads(msg.payload.decode('utf8'))
    get_topic = msg.topic
    ros_pub(get_topic, get_data)

def ros_pub(topic, data):
    global pub_location, gps_data_save, pub_service_mode
    print("topic::",topic)
    print("data::", data, type(data))
    print("==============================")
    latlng = NavSatFix()
    service_mode = UInt8()
    if(topic.find("/pending") > 1):
        service_mode.data = ServiceMode.Pending
        pub_service_mode.publish(service_mode)
        callDriveService(DriveMode.lock, "")
    elif(topic.find("/gps_summit") > 1):
        print("data::gps_summit",data)
        latlng.header.stamp = rospy.Time.now()
        latlng.header.frame_id = "world"
        latlng.latitude = float(data['od_data']['origin']['lat'])
        latlng.longitude = float(data['od_data']['origin']['lng'])
        # latlng.latitude = float(35.6479613028)
        # latlng.longitude = float(134.032092814)
        latlng.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        print(":::gps_summit:::latlng::",latlng)
        gps_data_save = data
        pub_location.publish(latlng)
        service_mode.data = ServiceMode.Auto
        pub_service_mode.publish(service_mode)
    elif(topic.find("/gps_confirm") > 1):
        latlng.header.stamp = rospy.Time.now()
        latlng.header.frame_id = "world"
        latlng.latitude = float(gps_data_save['od_data']['destination']['lat'])
        latlng.longitude = float(gps_data_save['od_data']['destination']['lng'])
        # latlng.latitude = float(35.6478800523)
        # latlng.longitude = float(134.032039782)
        latlng.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        print(":::gps_confirm:::latlng::",latlng)
        pub_location.publish(latlng)
        service_mode.data = ServiceMode.Auto
        pub_service_mode.publish(service_mode)

    elif(topic.find("/riding") > 1):
        print("data::riding",data)
        service_mode.data = ServiceMode.Manual
        pub_service_mode.publish(service_mode)
        callDriveService(DriveMode.release, "")
    elif(topic.find("/following") > 1):
        print("data::following",data)
        service_mode.data = ServiceMode.Following
        pub_service_mode.publish(service_mode)
        callDriveService(DriveMode.auto_mode, "tag_cmd_vel")
    elif(topic.find("/customize") > 1):
        service_mode.data = ServiceMode.Pending
        pub_service_mode.publish(service_mode)
        print("data::customize",data)
        # Setting device control
        # print(data['light_color'])
        get_data = data['light_color']
        get_data = list(get_data.replace("#",""))
        # print("get_data",get_data)
        r_data = int("0x"+get_data[0] + get_data[1],16)
        g_data = int("0x"+get_data[2] + get_data[3],16)
        b_data = int("0x"+get_data[4] + get_data[5],16)
        # print("r_data",r_data,g_data,b_data)

        if(data['hvac_active'] == True):
            msg_sended = Bool()
            msg_sended.data = True
            pub_hvac_front.publish(msg_sended)
            pub_hvac_back.publish(msg_sended)
        elif (data['hvac_active'] == False):
            msg_sended = Bool()
            msg_sended.data = False
            pub_hvac_front.publish(msg_sended)
            pub_hvac_back.publish(msg_sended)
        if(data['light_active'] == True):
            msg_sended = Bool()
            msg_sended.data = True
            ColorRGBA_msg = ColorRGBA()
            ColorRGBA_msg.r = r_data
            ColorRGBA_msg.g = g_data
            ColorRGBA_msg.b = b_data
            ColorRGBA_msg.a = 0
            pub_light_color.publish(ColorRGBA_msg)
        elif (data['light_active'] == False):
            msg_sended = Bool()
            msg_sended.data = False
            ColorRGBA_msg = ColorRGBA()
            ColorRGBA_msg.r = 0
            ColorRGBA_msg.g = 0
            ColorRGBA_msg.b = 0
            ColorRGBA_msg.a = 0
            pub_light_color.publish(ColorRGBA_msg)
            # 000 -> close
    else:
        print("no get setting topic")

def msg2json(msg):
    y = yaml.safe_load(str(msg))
    return json.dumps(y, indent=4)

def data_type(dataType):
    if dataType == "String":
        return String
    if dataType == "Twist":
        return Twist

def create_callback(subscriberInfo):
    # print(subscriberInfo)
    def function_template(data):
        #print dir(data)
        if subscriberInfo['type'] == 'String':
            # print("data",data)
            # print(msg2json(data))
            # mqttc.publish(subscriberInfo['mqtt-topic'], data.data)
            # mqtt_pub_topic = get_local_mac + subscriberInfo['mqtt-topic']
            # print (get_local_mac)
            client.publish(get_local_mac + "/" +
                           subscriberInfo['mqtt-topic'], msg2json(data))
        if subscriberInfo['type'] == 'Twist':
            # print("data",data,type(data))
            # print(msg2json(data))
            # mqttc.publish(subscriberInfo['mqtt-topic'], str(data.angular) + str(data.linear))
            # print get_local_mac
            # print subscriberInfo['mqtt-topic']
            client.publish(get_local_mac + "/" +
                           subscriberInfo['mqtt-topic'], msg2json(data))
    return function_template

def stop_all(*args):
    client.disconnect()

if __name__ == '__main__':
    global pub_location, config_folder, pub_service_mode,latlng_destination
    signal.signal(signal.SIGTERM, stop_all)
    signal.signal(signal.SIGQUIT, stop_all)
    signal.signal(signal.SIGINT,  stop_all)  # Ctrl-C

    rospy.init_node('mqtt_bridge_node', anonymous=True)
    config_folder = rospy.get_param("~config_folder", "")
    pub_location = rospy.Publisher(
        'pev_gps_destination', NavSatFix, queue_size=10)
    pub_service_mode = rospy.Publisher(
        'service_mode', UInt8, queue_size=10, latch=True)
    
    pub_hvac_front = rospy.Publisher(
        'hvac_front', Bool, queue_size=10, latch=True)
    pub_hvac_back = rospy.Publisher(
        'hvac_2_relay', Bool, queue_size=10, latch=True)
    pub_light_check = rospy.Publisher(
        'light_check', Bool, queue_size=10, latch=True)
    pub_light_color = rospy.Publisher(
        'custom_led',ColorRGBA,queue_size=10,latch=True)

    ros_jsonFile = open(config_folder + "ros_config.json")
    ros_jsonStr = ros_jsonFile.read()
    ros_jsonData = json.loads(ros_jsonStr)
    config_jsonFile = open(config_folder + "config.json")
    config_jsonFile_Str = config_jsonFile.read()
    config_jsonData = json.loads(config_jsonFile_Str)

    callbacks = []
    for idx, subscribers in enumerate(ros_jsonData['subscribers']):
        # print("subscribers",ros_jsonData['subscribers'])
        # print(idx, subscribers)
        callbacks.append(create_callback(subscribers))
        rospy.Subscriber(
        subscribers['ros-topic'], data_type(subscribers['type']), callbacks[idx])

    client = mqtt.Client()
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(config_jsonData['cloud_mqtt_broker_address'],
                   config_jsonData['cloud_mqtt_broker_port'], 60)
    client.loop_forever()
