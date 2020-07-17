#!/usr/bin/env python
import os
import cv2
import rospy
import math
import numpy as np
import tf
import PIL.Image as Image
import PIL.ImageOps as ImageOps
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from campusrover_msgs.srv import *
from button_recognition.srv import *
from cv_bridge import CvBridge
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from campusrover_msgs.msg import ButtonCommand

global init_brightness_value
global check_brightness_value

class ButtonTracker:
  def __init__(self):
    self.detected_box = None
    self.tracker = None

  def init_tracker(self, image, box_list):
    self.tracker = None
    self.tracker = cv2.MultiTracker_create()
    for box_item in box_list:
      self.tracker.add(cv2.TrackerKCF_create(), image, tuple(box_item))

  def call_for_service(self,image):
    rospy.wait_for_service('recognition_service')
    rosimage = Image()
    rosimage.header.stamp = rospy.Time.now()
    rosimage.data = np.array(cv2.imencode('.jpg', image)[1]).tostring()
    try:
      recognize = rospy.ServiceProxy('recognition_service', recog_server)
      response = recognize(rosimage)
      if response is None:
        print("None service response!")
      boxes, scores, texts, beliefs = [], [], [], []

      for pred in response.box.data:
        boxes.append([pred.x_min,  pred.y_min, pred.x_max, pred.y_max])
        scores.append(pred.score)
        text = pred.text
        texts.append(text.replace(' ', ''))
        beliefs.append(pred.belief)
      return boxes, scores, texts, beliefs
    except rospy.ServiceException(e):
      print("recognition service failed: {}".format(e))
  
  def visualize_recognitions(self,frame, box, text):
    # draw bounding boxes
    p1, p2 = (int(box[0]), int(box[1])), (int(box[2]), int(box[3]))
    cv2.rectangle(frame, p1, p2, (50, 220, 100), thickness=2)
    # draw text at a proper location
    btn_width = (box[2] - box[0]) / 2.0
    btn_height = (box[3] - box[1]) / 2.0
    font_size = min(btn_width, btn_height) * 0.6
    text_len = len(text)
    font_pose = int(0.5*(box[0]+box[2]) - 0.5 * text_len * font_size), int(0.5*(box[1]+box[3]) + 0.5 * font_size)
    # font_pose is the bottom_left of the text
    cv2.putText(frame, text, font_pose, cv2.FONT_HERSHEY_SIMPLEX, 0.6, thickness=2, color=(255, 0, 255))

class read_video_and_recognize:
  def __init__(self):
    self.boxes=[]
    self.box=[0,0,0,0]
    self.texts=[]
    self.recognize_check=False
    self.pixel_depth_ros=1
    self.x_biase=0
    self.y_biase=0
    self.button_info=''
    self.presscheck=False
    self.frame_id=''
    self.hsvcheck=False
    self.button_status=''
    self.button_status_check=False
  def read_and_recognize(self,Image):
  # initialize tracking process
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(Image, 'bgr8') 
    self.frame_id=Image.header.frame_id
    print(self.frame_id)
    button_tracker = ButtonTracker()
    if self.recognize_check == True:  
      (self.boxes, scores, self.texts, beliefs) = button_tracker.call_for_service(cv_image)
      for box, text in zip(self.boxes, self.texts):
    # output video in ros
        button_tracker.visualize_recognitions(cv_image, box, text)
        ros_result_image=bridge.cv2_to_imgmsg(cv_image,'bgr8')
        pub.publish(ros_result_image)
      self.recognize_check = False
  
  def imagetohsv (self,Image):
    global init_brightness_value
    global check_brightness_value
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(Image, 'bgr8')
    hsv=cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
    if self.hsvcheck == True:
      x=self.box[0]
      y=self.box[1]
      w=self.box[2]-self.box[0]
      h=self.box[3]-self.box[1]
      button_image_array = hsv[y:y+h, x:x+w]
      # cv2.imshow("button", button_image_array)
      if self.button_status == 'init':
        init_brightness_value=np.sum(button_image_array[:,:,2])/np.size(button_image_array[:,:,2])
        self.hsvcheck = False
      if self.button_status == 'check':
        check_brightness_value=np.sum(button_image_array[:,:,2])/np.size(button_image_array[:,:,2])
        diff_brightness=check_brightness_value-init_brightness_value
        if diff_brightness > brightness_set :
          self.button_status_check = True
        else:
          self.button_status_check = False
        self.hsvcheck = False
              
      
    


  def depth_image(self,data):
    bridge = CvBridge()
    cv_depth_image = bridge.imgmsg_to_cv2(data, '16UC1')
    i=0
    text='set'
    presstext='set'
    for text in self.texts:
      if text==self.button_info:
        self.box=self.boxes[i]
        presstext=text
        break
      else:
        i=i+1
      if i==len(self.texts):
        self.recognize_check = True
    point_x = (self.box[2] + self.box[0]) / 2.0 
    point_y = (self.box[3] + self.box[1]) / 2.0
    pixel_depth=cv_depth_image[int(point_y),int(point_x)]
   
    self.pixel_depth_ros=float(pixel_depth)/1000
    pixel_diff_y=int(point_y)-239.967
    pixel_diff_x=int(point_x)-325.548

    # calculation each pixel angle
    # each_pixel_angle_x=math.radians(float(55)/float(640))
    # each_pixel_angle_y=math.radians(float(43)/float(480))
    # pixel_angle_x=pixel_diff_x*each_pixel_angle_x
    # pixel_angle_y=pixel_diff_y*each_pixel_angle_y
    # x_biase=self.pixel_depth_ros*math.tan(pixel_angle_x)
    # y_biase=self.pixel_depth_ros*math.tan(pixel_angle_y)

    # calculation the real image longth
    x=2*self.pixel_depth_ros*math.tan(math.radians(54/2))
    y=2*self.pixel_depth_ros*math.tan(math.radians(43/2))
    diff_x=x/640
    diff_y=y/480
    self.x_biase=diff_x*pixel_diff_x
    self.y_biase=diff_y*pixel_diff_y

    goal = PoseStamped()
    goal.header.seq = 1
    goal.header.stamp = rospy.Time.now()
    goal.header.frame_id = self.frame_id
    goal.pose.position.x = self.x_biase
    goal.pose.position.y = self.y_biase
    goal.pose.position.z = self.pixel_depth_ros
    if self.pixel_depth_ros>0 and self.pixel_depth_ros<0.5 and presstext == self.button_info and self.presscheck == True and self.button_status == 'init':
      read=read_video_and_recognize()
      read.call_arm_service(goal)
      self.presscheck = False
    if presstext == self.button_info and self.button_status == 'check':
      read=read_video_and_recognize()
      read.call_arm_service_check(self.button_status_check)
      self.presscheck = False
    

  def button_info_enable(self,button):
    self.button_info=button.button_name.data
    self.button_status=button.command_type.data
    self.presscheck = True
    self.recognize_check=True
    self.hsvcheck = True

  def call_arm_service(self,pose_data):
    rospy.wait_for_service('arm_action')
    try:
      pose = rospy.ServiceProxy('arm_action', ArmAction)
      response_ans = pose(pose_data)
    except rospy.ServiceException(f):
      print("arm_action service failed: {}".format(f))
  
  def call_arm_service_check(self,buttonstatus_data):
    rospy.wait_for_service('button_status')
    try:
      status = rospy.ServiceProxy('button_status', ButtonStatus)
      response_ans = status(buttonstatus_data)
    except rospy.ServiceException(f):
      print("button_status service failed: {}".format(f))
  
  

if __name__ == '__main__':
  rospy.init_node('button_tracker', anonymous=True)
  read=read_video_and_recognize()
  brightness_set= rospy.get_param('/brightness_detect',10)
  pub=rospy.Publisher('button_recognize_image',Image,queue_size=2)
  rospy.Subscriber('/aligned_depth_image_raw',Image,read.depth_image)
  
  rospy.Subscriber("/color_image_raw", Image,read.read_and_recognize)
  rospy.Subscriber("/color_image_raw", Image,read.imagetohsv)
  rospy.Subscriber('/button_info', ButtonCommand,read.button_info_enable)
  rospy.spin()
