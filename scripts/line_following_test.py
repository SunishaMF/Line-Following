#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np

from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Image, CameraInfo

class Follower:
  def __init__(self):

    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber('rgb_image', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd_manual',
                                       CarlaEgoVehicleControl, queue_size=1)
    self.control = CarlaEgoVehicleControl()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    converted_image = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)
    #White color mask
    lower_threshold = np.uint8([0, 200, 0])
    upper_threshold = np.uint8([255, 255, 255])
    white_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    #Red color mask
    lower_threshold = np.uint8([0, 160, 160])
    upper_threshold = np.uint8([140, 255, 255])
    red_mask = cv2.inRange(converted_image, lower_threshold, upper_threshold)
    
    #Combine white and yellow masks
    mask = cv2.bitwise_or(white_mask, red_mask)
    masked_image = cv2.bitwise_and(image, image, mask = mask)
    h, w, d = image.shape
    M = cv2.moments(masked_image)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      cv2.circle(image, (cx,cy), 20, (0,0,0), -1)


      err = cx - w/2
      self.control.throttle = 0.15
      self.control.steer = -float(err) / 100
      self.cmd_vel_pub.publish(self.control)
    

    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)