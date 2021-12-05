#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
import numpy as np
from typing import Optional
from carla_msgs.msg import CarlaEgoVehicleControl
from sensor_msgs.msg import Image, CameraInfo
#from geometry_msgs.msg import Twist


class Follower:
    def __init__(self):
        self.bridge = CvBridge()
    
        self.image_sub = rospy.Subscriber('rgb_image', 
                                        Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/carla/ego_vehicle/vehicle_control_cmd_manual',
                                        CarlaEgoVehicleControl, queue_size=1)
        self.control = CarlaEgoVehicleControl()

        self.rgb_lower_range = np.array([0, 160, 160])
        self.rgb_upper_range = np.array([140, 255, 255])
        self.ycbcr_lower_range = np.array([0, 160, 160])
        self.ycbcr_upper_range = np.array([140, 255, 255])
        

    def image_callback(self, msg):
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        self.cmd_vel_pub.publish(self.control)

    
    def find_error(self):
        # make rgb and depth into the same shape
        data: np.ndarray = cv2.resize(self.Image.copy(),
                                    dsize=(192, 256))
        # cv2.imshow("rgb_mask", cv2.inRange(data, self.rgb_lower_range, self.rgb_upper_range))
        data = self.rgb2ycbcr(data)
        # cv2.imshow("ycbcr_mask", cv2.inRange(data, self.ycbcr_lower_range, self.ycbcr_upper_range))
        # find the lane
        error_at_10 = self.find_error_at(data=data,
                                        y_offset=10,
                                        lower_range=self.ycbcr_lower_range,
                                        upper_range=self.ycbcr_upper_range,
                                        error_scaling=[
                                            (20, 0.1),
                                            (40, 0.75),
                                            (60, 0.8),
                                            (80, 0.9),
                                            (100, 0.95),
                                            (200, 1)
                                        ])
        error_at_50 = self.find_error_at(data=data,
                                        y_offset=50,
                                        lower_range=self.ycbcr_lower_range,
                                        upper_range=self.ycbcr_upper_range,
                                        error_scaling=[
                                            (20, 0.2),
                                            (40, 0.4),
                                            (60, 0.7),
                                            (70, 0.7),
                                            (80, 0.7),
                                            (100, 0.8),
                                            (200, 0.8)
                                        ]
                                        )
        if error_at_10 is None and error_at_50 is None:
            return None

        # we only want to follow the furthest thing we see.
        error = 0
        if error_at_10 is not None:
            error = error_at_10
        if error_at_50 is not None:
            error = error_at_50
        return error
    
    def find_error_at(self, data, y_offset, error_scaling, lower_range, upper_range) -> Optional[float]:
        y = data.shape[0] - y_offset
        lane_x = []
        cv2.imshow("data", data)
        # mask_red = cv2.inRange(src=data, lowerb=(0, 150, 60), upperb=(250, 240, 140))  # TERRACE RED
        # mask_yellow = cv2.inRange(src=data, lowerb=(0, 130, 0), upperb=(250, 200, 110)) # TERRACE YELLOW
        mask_red = cv2.inRange(src=data, lowerb=(0, 180, 60), upperb=(250, 240, 140))  # CORY 337 RED
        mask_yellow = cv2.inRange(src=data, lowerb=(0, 140, 0), upperb=(250, 200, 80))  # CORY 337 YELLOW
        # mask = mask_yellow
        mask = mask_red | mask_yellow

        # cv2.imshow("mask_red", mask_red)
        # cv2.imshow("mask_yellow", mask_yellow)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=1)
        h, w, d = data.shape
        M = cv2.moments(mask)
        if M['m00'] > 0:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            


            err = cx - w/2
            self.control.throttle = 0.15
            self.control.steer = -float(err) / 100
            self.cmd_vel_pub.publish(self.control)
            

    
        cv2.imshow("mask", mask)

        # for x in range(0, data.shape[1], 5):
        #     if mask[y][x] > 0:
        #         lane_x.append(x)

        # if len(lane_x) == 0:
        #     return None

        # # if lane is found
        # avg_x = int(np.average(lane_x))

        # # find error
        # center_x = data.shape[1] // 2

        # error = avg_x - center_x
        # # we want small error to be almost ignored, only big errors matter.
        # for e, scale in error_scaling:
        #     if abs(error) <= e:
        #         print(f"Error at {y_offset} -> {error, scale} -> {error * scale}")
        #         error = error * scale
        #         break



    def rgb2ycbcr(self, im):
        xform = np.array([[.299, .587, .114],
                          [-.1687, -.3313, .5],
                          [.5, -.4187, -.0813]])
        ycbcr = im.dot(xform.T)
        ycbcr[:, :, [1, 2]] += 128
        return np.uint8(ycbcr)

rospy.init_node('follower')
follower = Follower()
rospy.spin()
