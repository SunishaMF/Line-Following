#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np


from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist


class Follower:
  def __init__(self):

    self.bridge = CvBridge()
    
    self.image_sub = rospy.Subscriber('rgb_image', 
                                      Image, self.image_callback)
    self.cmd_vel_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd_manual',
                                       Twist, queue_size=1)
    self.twist = Twist()

  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    lower_yellow = np.array([ 50, 50, 170])
    upper_yellow = np.array([255, 255, 190])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = 3*h/4
    search_bot = search_top + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])


      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
    

    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)


rospy.init_node('follower')
follower = Follower()
rospy.spin()
