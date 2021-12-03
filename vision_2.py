#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image

# imports from image files
import roslib
import sys
import cv2
import numpy as np
from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray, Float64
from cv_bridge import CvBridge, CvBridgeError


class vision2:

  # Defines publisher and subscriber
  def __init__(self):

    rospy.init_node('image_processing', anonymous=True)
    self.bridge = CvBridge()
    self.cv_image1 = Image()
    self.cv_image2 = Image()
    self.joint1 = Float64()
    self.joint1_pub = rospy.Publisher('joint_angle_1', Float64, queue_size = 10)
    self.joint3_pub = rospy.Publisher('joint_angle_3', Float64, queue_size = 10)
    self.joint4_pub = rospy.Publisher('joint_angle_4', Float64, queue_size = 10)

    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1)
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)


    

    	
  def detect_red(self,image):
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      height, width = image.shape[:2]
      if height == 0:
          return np.array([-1, -1])
      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      
      height, width = image.shape[:2]
      if height == 0:
          return np.array([-1, -1])
      
      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      height, width = image.shape[:2]

      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      height, width = image.shape[:2]
      if height == 0:
          return np.array([-1, -1])
      
      M = cv2.moments(mask)
      if M['m00'] == 0:
          return np.array([-1, -1])
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


      


  def pixel2meter(self,image):
      circle1Pos = self.detect_red(image)
      circle2Pos = self.detect_green(image)
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 10 / np.sqrt(dist)
  


  def detect_joint_angles(self,image1, image2, prev_estimate):
    a = self.pixel2meter(image1)
    a2 = self.pixel2meter(image2)
    yellow1 = self.detect_yellow(image1) 
    blue1 = self.detect_blue(image1) 
    red1 = self.detect_red(image1)
    yellow2 = self.detect_yellow(image2) 
    blue2 = self.detect_blue(image2) 
    red2 =  self.detect_red(image2)

    
    ja3_camera1 = np.arctan2(yellow1[0] - blue1[0], yellow1[1] - blue1[1])
    ja3_camera2 = np.arctan2(yellow2[0] - blue2[0], yellow2[1] - blue2[1])
    ja4_camera1 = np.arctan2(blue1[0] - red1[0], blue1[1] - red1[1])
    ja4_camera2 = np.arctan2(blue2[0] - red2[0], blue2[1] - red2[1])
    
    ja3 = prev_estimate
    
    if ((ja3_camera1 == 0) or (np.arctan(ja3_camera2/ja3_camera1) == 1) or (np.abs(np.arctan(ja3_camera2/ja3_camera1)) > np.pi) and (ja4_camera2 != 0)):
        if (ja4_camera1 < 0):
            if (ja4_camera2 < 0):

                ja1 = - (np.arctan(ja4_camera1/ja4_camera2) + np.pi)
            if (ja3_camera2 > 0):

                ja1 = - (np.arctan(ja4_camera1/ja4_camera2) - np.pi)
        else:
            if (ja3_camera1 > 0):
                if (ja3_camera2 < 0):

                    ja1 = - np.arctan(ja4_camera1/ja4_camera2) 
                if (ja3_camera2 > 0):

                    ja1 = - np.arctan(ja4_camera1/ja4_camera2)
    else:
        if (ja3_camera1 < 0):
            if (ja3_camera2 < 0):

                ja1 = - (np.arctan(ja3_camera2/ja3_camera1) - np.pi)
            if (ja3_camera2 > 0):
 
                ja1 = - (np.arctan(ja3_camera2/ja3_camera1) + np.pi)
        else:
            if (ja3_camera1 > 0):
                if (ja3_camera2 < 0):
 
                    ja1 = - (np.arctan(ja3_camera2/ja3_camera1) + np.pi)
                if (ja3_camera2 > 0):

                    ja1 = - np.arctan(ja3_camera2/ja3_camera1)


    if (np.abs(ja1 - prev_estimate) > 0.25):
        if (np.abs(ja1 - np.pi - prev_estimate) < 0.4):
            ja1 = ja1 - np.pi
        if (np.abs(ja1 + np.pi - prev_estimate) < 0.4):
            ja1 = ja1 + np.pi
        if (np.abs(-ja1 - prev_estimate) < 0.4):
            ja1 = -ja1
        
        
    ja3 = np.arctan2(yellow1[0] - blue1[0], yellow1[1] - blue1[1]) * np.sin(ja1)
    + (np.arctan2(yellow2[0] - blue2[0], yellow2[1] - blue2[1])) * np.cos(ja1)


    ja4 = (np.arctan2(blue1[0] - red1[0], blue1[1] - red1[1]) * np.sin(ja1)
    + np.arctan2(blue2[0] - red2[0], blue2[1] - red2[1]) * np.cos(ja1)) - ja3
    


    return np.array([ja1, ja3, ja4])

  
  def callback1(self, data): 
      try:
            self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
            print(e)
      


  def callback2(self, data): 
      try:
            self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
      except CvBridgeError as e:
            print(e)
      
      self.callback()




  def callback(self):
        
    
        angles = self.detect_joint_angles(self.cv_image1, self.cv_image2, self.joint1.data)

        self.joint1 = Float64()
        self.joint1.data = angles[0]
        
        self.joint3 = Float64()
        self.joint3.data = angles[1]
    
        self.joint4 = Float64()
        self.joint4.data = angles[2]
    
        try:

            self.joint1_pub.publish(self.joint1)
            self.joint4_pub.publish(self.joint4)
            self.joint3_pub.publish(self.joint3)
        except CvBridgeError as e:
            print(e)
 
    
def main(args):
  vis2 = vision2()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
    
