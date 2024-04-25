#!/usr/bin/env python3
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
from skimage.feature import hog
from skimage import color
from joblib import load



class identify_sign(Node):
    def __init__(self):
        super().__init__('identify_sign')
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

        # self.img_publisher = self.create_publisher(CompressedImage, '/identify_sign/compressed', 10)
        self.state_publisher = self.create_publisher(Point, '/state', 10)
        self.state = Point()
        self.clf = load('/home/ysunil3/turtlebot3_ws/src/bowser_jr_object_follower/Lab 6/svm_classifier_model.joblib')
        self.svmclf = load('/home/ysunil3/turtlebot3_ws/src/bowser_jr_object_follower/Lab 6/svm_ycroppede_classifier_model.joblib')
        self.MLPclf = load('/home/ysunil3/turtlebot3_ws/src/bowser_jr_object_follower/Lab 6/MLP_ycropped_classifier_model.joblib')
        self.count = 0.0
        # self.blueLower = (100,150,50)
        # self.blueUpper = (140,255,255)
        

    def _image_callback(self, CompressedImage):	
        features = self.preprocess_image(CompressedImage)
        features = np.array([features])
        featuresog = self.preprocess_image_og(CompressedImage)
        featuresog = np.array([featuresog])
        y_pred_og = self.clf.predict(featuresog)
        y_pred_svm = self.svmclf.predict(features)
        y_pred_MLP = self.MLPclf.predict(features)
        # print("og", y_pred_og) 
        # print("svm", y_pred_svm)        
        # print("MLP", y_pred_MLP)
        if y_pred_svm == 1: # left
            self.state.x = 3.0
        elif y_pred_svm == 2: # right
            self.state.x = 4.0
        elif y_pred_svm == 3 or y_pred_svm == 4: # u-turn or stop
            self.state.x = 5.0
        elif y_pred_svm == 5 or y_pred_MLP == 0: # goal or wall
            self.state.x = 6.0
        self.state.y = self.count
        self.count += 1.0
        # print("state", self.state.x)
        # print("prediction number", self.count)
        # print("sign", y_pred)
        # print("state", self.state.x)
        # self.get_logger().info('pred num:{}'.format(self.count))
        # self.get_logger().info('sign:{}'.format(y_pred))
        # self.get_logger().info('state:{}'.format(self.state.x))

        self.state_publisher.publish(self.state)
        # The "CompressedImage" is transformed to a color image in BGR space and is store in "_imgBGR"
        # self._imgBGR = CvBridge().compressed_imgmsg_to_cv2(CompressedImage, "bgr8")
        # blurred = cv2.GaussianBlur(self._imgBGR, (11, 11), 0)
        # hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        # mask = cv2.inRange(hsv, self.blueLower, self.blueUpper)
        # mask = cv2.erode(mask, None, iterations=2)
        # mask = cv2.dilate(mask, None, iterations=2)
        
        # cnts, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
        #                         cv2.CHAIN_APPROX_SIMPLE)
        # center = None
        # if len(cnts) > 0:
        #     c = max(cnts, key=cv2.contourArea)
        #     ((x,y), radius) = cv2.minEnclosingCircle(c)

        #     if radius > 10:
        #         cv2.circle(self._imgBGR, (int(x), int(y)), int(radius),
        #                         (0, 255, 255), 2)
        #         cv2.circle(self._imgBGR, (int(x), int(y)), 5, (0, 0, 255,), -1)
        #         self.centroid.x = x
        #         self.centroid.y = y
        #         self.centroid.z = 0.0
        #         self.centroid_publisher.publish(self.centroid)
        # else:
        #     self.centroid.x = 0.0
        #     self.centroid.y = 0.0
        #     self.centroid.z = 44.0
        #     self.centroid_publisher.publish(self.centroid)
        # self._imgBGR = CvBridge().cv2_to_compressed_imgmsg(self._imgBGR, dst_format='jpeg')

        # self.img_publisher.publish(self._imgBGR)
		
    def preprocess_image(self, image):
        # Load the image
        img = CvBridge().compressed_imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        x, y, w, h = 12, 12, 40, 40
        img = cv2.resize(img, (64, 64))
        gray_image = color.rgb2gray(img[y:y+h, :])
        hsv_image = cv2.cvtColor(img[y:y+h, :], cv2.COLOR_RGB2HSV)
        cv2.imshow('frame',img[y:y+h, :])
        if(self._display_image):
            # Display the image in a window
            self.show_image(img[y:y+h, :])
        # gray_image = cv2.resize(gray_image, (64, 64))
        hog_features = hog(gray_image, orientations=8, pixels_per_cell=(8, 8),
                        cells_per_block=(2, 2), block_norm='L2', feature_vector=True)
        hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        hsv_image = hsv_image.flatten()  
        hsv_image = hsv_image / 255.0  # norm 

        sobelx = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3) 
        sobely = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3) 
        sobel_edges = np.hypot(sobelx, sobely) 
        edges_resized = 30 * sobel_edges.flatten() / 255

        features = np.hstack([hog_features, hsv_image, edges_resized]) 
        return features
    def preprocess_image_og(self, image):
        # Load the image
        img = CvBridge().compressed_imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # x, y, w, h = 12, 12, 40, 40
        img = cv2.resize(img, (64, 64))
        gray_image = color.rgb2gray(img)
        hsv_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        # gray_image = cv2.resize(gray_image, (64, 64))
        hog_features = hog(gray_image, orientations=8, pixels_per_cell=(8, 8),
                        cells_per_block=(2, 2), block_norm='L2', feature_vector=True)
        hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0)
        hsv_image = hsv_image.flatten()  
        hsv_image = hsv_image / 255.0  # norm 

        features = np.hstack([hog_features, hsv_image]) 
        return features

    def show_image(self, img):
        cv2.imshow(self._titleOriginal, img)
        # Cause a slight delay so image is displayed
        self._user_input=cv2.waitKey(50) #Use OpenCV keystroke grabber for delay.

    def get_user_input(self):
        return self._user_input

def main():
	rclpy.init() #init routine needed for ROS2.
	identify_sign_node = identify_sign() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(identify_sign_node) # Trigger callback processing.
		if(identify_sign_node._display_image):	
			if identify_sign_node.get_user_input() == ord('q'):
				cv2.destroyAllWindows()
				break

	#Clean up and shutdown.
	identify_sign_node.destroy_node()  
	rclpy.shutdown()


if __name__ == '__main__':
	main()
