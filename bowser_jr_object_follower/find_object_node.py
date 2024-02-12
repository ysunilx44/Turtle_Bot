#!/usr/bin/env python3
import numpy as numpy
import cv2
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import sys
from cv_bridge import CvBridge
import numpy as np
from geometry_msgs.msg import Point


class find_object(Node):
	def __init__(self):
		super().__init__('find_object')
		# Set Parameters
		self.declare_parameter('show_image_bool', False)
		self.declare_parameter('window_name', "Raw Image")

		#Determine Window Showing Based on Input
		self._display_image = bool(self.get_parameter('show_image_bool').value)

		# Declare some variables
		self._titleOriginal = self.get_parameter('window_name').value # Image Window Title	
		
		#Only create image frames if we are not running headless (_display_image sets this)
		if(self._display_image):
		# Set Up Image Viewing
			cv2.namedWindow(self._titleOriginal, cv2.WINDOW_AUTOSIZE ) # Viewing Window
			cv2.moveWindow(self._titleOriginal, 50, 50) # Viewing Window Original Location
		
		#Set up QoS Profiles for passing images over WiFi
		image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			durability=QoSDurabilityPolicy.VOLATILE,
			depth=1
		)

		#Declare that the minimal_video_subscriber node is subcribing to the /camera/image/compressed topic.
		self.find_object_subscriber = self.create_subscription(
				CompressedImage,
				'/image_raw/compressed',
				self._image_callback,
				image_qos_profile)
		self.find_object_subscriber # Prevents unused variable warning.

		self.img_publisher = self.create_publisher(CompressedImage, '/find_object/compressed', 10)
		self.centroid_publisher = self.create_publisher(Point, '/object_centroid', 10)

		# self.blueLower = (100,150,50)
		# self.blueUpper = (140,255,255)
		self.blueball = np.uint8([[[79,173,206]]])
		self.hsv_color = cv2.cvtColor(self.blueball,cv2.COLOR_RGB2HSV) # NOTE: THIS IS NOW RGB2HSV
		self.blueLower = np.array([self.hsv_color[0][0][0]-20, self.hsv_color[0][0][1]*(150/255), self.hsv_color[0][0][2]*(50/255)], np.uint8)
		self.blueUpper = np.array([self.hsv_color[0][0][0]+20, 255, 255], np.uint8)
		self.centroid = Point()

	def _image_callback(self, CompressedImage):	
		# The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
		self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
		blurred = cv2.GaussianBlur(self._imgBGR, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		mask = cv2.inRange(hsv, self.blueLower, self.blueUpper)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		
		cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
								cv2.CHAIN_APPROX_SIMPLE)
		center = None
		if len(cnts) > 0:
			c = max(cnts, key=cv2.contourArea)
			((x,y), radius) = cv2.minEnclosingCircle(c)

			if radius > 10:
				cv2.circle(self._imgBGR, (int(x), int(y)), int(radius),
								(0, 255, 255), 2)
				cv2.circle(self._imgBGR, (int(x), int(y)), 5, (0, 0, 255,), -1)
				self.centroid.x = x
				self.centroid.y = y
				self.centroid.z = 0.0
				self.centroid_publisher.publish(self.centroid)
		else:
			self.centroid.x = 0.0
			self.centroid.y = 0.0
			self.centroid.z = 44.0
			self.centroid_publisher.publish(self.centroid)
		self._imgBGR = CvBridge().cv2_to_compressed_imgmsg(self._imgBGR, dst_format='jpeg')

		self.img_publisher.publish(self._imgBGR)
		

		#print("Centroid X: %0.2f Centroid Y: %0.2f" % (x, y))
		#cv2.imshow('frame',self._imgBGR)
	# 	if(self._display_image):
	# 		# Display the image in a window
	# 		self.show_image(self._imgBGR)

	# def show_image(self, img):
	# 	cv2.imshow(self._titleOriginal, img)
	# 	# Cause a slight delay so image is displayed
	# 	self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

	# def get_user_input(self):
	# 	return self._user_input


def main():
	rclpy.init() #init routine needed for ROS2.
	find_object_subscriber = find_object() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(find_object_subscriber) # Trigger callback processing.
		if(find_object_subscriber._display_image):	
			if find_object_subscriber.get_user_input() == ord('q'):
				cv2.destroyAllWindows()
				break

	#Clean up and shutdown.
	find_object_subscriber.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
