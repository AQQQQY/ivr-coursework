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


class vision1:

  # Defines publisher and subscriber
  def __init__(self):
    rospy.init_node('image_processing', anonymous=True)
    # self.joints_pub = rospy.Publisher("joints_pos",Float64MultiArray, queue_size=10)
    self.joint2_pub = rospy.Publisher('joint_angle_2', Float64, queue_size = 10)
    self.joint3_pub = rospy.Publisher('joint_angle_3', Float64, queue_size = 10)
    self.joint4_pub = rospy.Publisher('joint_angle_4', Float64, queue_size = 10)
    
    self.bridge = CvBridge()   
    self.image_pub1 = rospy.Publisher("image_topic1",Image, queue_size = 1) 
    self.image_sub1 = rospy.Subscriber("/camera1/robot/image_raw",Image,self.callback1)
    self.image_pub2 = rospy.Publisher("image_topic2",Image, queue_size = 1)
    self.image_sub2 = rospy.Subscriber("/camera2/robot/image_raw",Image,self.callback2)
    
    

    
    	
  
  def detect_red(self,image):
      mask = cv2.inRange(image, (0, 0, 100), (0, 0, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])



  def detect_green(self,image):
      mask = cv2.inRange(image, (0, 100, 0), (0, 255, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])


  def detect_blue(self,image):
      mask = cv2.inRange(image, (100, 0, 0), (255, 0, 0))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])

  def detect_yellow(self,image):
      mask = cv2.inRange(image, (0, 100, 100), (0, 255, 255))
      kernel = np.ones((5, 5), np.uint8)
      mask = cv2.dilate(mask, kernel, iterations=3)
      M = cv2.moments(mask)
      cx = int(M['m10'] / M['m00'])
      cy = int(M['m01'] / M['m00'])
      return np.array([cx, cy])



  def pixel2meter(self,image):
      circle1Pos = self.detect_red(image)
      circle2Pos = self.detect_green(image)
      dist = np.sum((circle1Pos - circle2Pos)**2)
      return 10 / np.sqrt(dist)

  def detect_joint_angles(self,image):
    a = self.pixel2meter(image)
    center = a * self.detect_green(image)
    circle1Pos = a * self.detect_yellow(image) 
    circle2Pos = a * self.detect_blue(image) 
    circle3Pos = a * self.detect_red(image)
    ja2 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1])
    ja3 = np.arctan2(circle1Pos[0]-circle2Pos[0], circle1Pos[1]-circle2Pos[1])
    ja4 = np.arctan2(circle2Pos[0]-circle3Pos[0], circle2Pos[1]-circle3Pos[1]) - ja2
    return np.array([ja2, ja3, ja4])
  
  def callback1(self,data):
    # Recieve the image
    try:
      self.cv_image1 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    
    a = self.detect_joint_angles(self.cv_image1)
    self.joint2 = Float64()
    self.joint2.data = a[0]   
    self.joint3 = Float64()
    self.joint3.data = a[1]   
    self.joint4 = Float64()
    self.joint4.data = a[2]
    

    try: 
      self.image_pub1.publish(self.bridge.cv2_to_imgmsg(self.cv_image1, "bgr8"))
      # self.joints_pub.publish(self.joints)
      self.joint2_pub.publish(self.joint2)
      self.joint3_pub.publish(self.joint3)
      self.joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)
      
  # Recieve data from camera 2, process it, and publish
  def callback2(self,data):
    # Recieve the image
    try:
      self.cv_image2 = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)
    a = self.detect_joint_angles(self.cv_image2)
    image = np.concatenate((self.cv_image1, self.cv_image2), axis=1)
    im = cv2.imshow('camera1 and camera 2', image)
    cv2.waitKey(1)
    
    self.joint2 = Float64()
    self.joint2.data = a[0]
    self.joint3 = Float64()
    self.joint3.data = a[1]
    self.joint4 = Float64()
    self.joint4.data = a[2]

    try: 
      self.image_pub2.publish(self.bridge.cv2_to_imgmsg(self.cv_image2, "bgr8"))
      self.joint2_pub.publish(self.joint2)
      self.joint3_pub.publish(self.joint3)
      self.joint4_pub.publish(self.joint4)
    except CvBridgeError as e:
      print(e)
      

# call the class
def main(args):
  vis1 = vision1()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

# run the code if the node is called
if __name__ == '__main__':
    main(sys.argv)
