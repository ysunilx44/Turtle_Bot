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
        self.clf = load('/home/ysunil3/turtlebot3_ws/src/bowser_jr_object_follower/Lab 6/svm_rm_classifier_model.joblib')
        self.svmclf = load('/home/ysunil3/turtlebot3_ws/src/bowser_jr_object_follower/Lab 6/svm_no_wall_classifier_model.joblib')
        self.MLPclf = load('/home/ysunil3/turtlebot3_ws/src/bowser_jr_object_follower/Lab 6/MLP_ycropped_classifier_model.joblib')
        self.count = 0.0
        # self.blueLower = (100,150,50)
        # self.blueUpper = (140,255,255)
        

    def _image_callback(self, CompressedImage):	
        features = self.preprocess_image(CompressedImage)
        features = np.array([features])
        featuresrm = self.preprocess_imagerm(CompressedImage)
        featuresrm = np.array([featuresrm])
        y_pred_rm = self.clf.predict(featuresrm)
        y_pred_svm = self.svmclf.predict(features)
        y_pred_MLP = self.MLPclf.predict(features)
        print("rm", y_pred_rm) 
        print("svm", y_pred_svm)        
        print("MLP", y_pred_rm)
        if y_pred_rm == 1: # left
            self.state.x = 3.0
        elif y_pred_rm == 2: # right
            self.state.x = 4.0
        elif y_pred_rm == 5: # goal 
            self.state.x = 6.0
        elif y_pred_svm == 3 or y_pred_rm == 4: # u-turn or stop
            self.state.x = 5.0
        elif y_pred_MLP == 0: # wall
            self.state.x = 6.0
        self.state.y = self.count
        self.count += 1.0
        print("state", self.state.x)

        # print("prediction number", self.count)
        # print("sign", y_pred)
        # print("state", self.state.x)
        # self.get_logger().info('pred num:{}'.format(self.count))
        # self.get_logger().info('sign:{}'.format(y_pred))
        # self.get_logger().info('state:{}'.format(self.state.x))

        self.state_publisher.publish(self.state)

		
    def preprocess_image(self, image):
        # Load the image
        img = CvBridge().compressed_imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        x, y, w, h = 12, 12, 40, 40
        img = cv2.resize(img, (64, 64))
        gray_image = color.rgb2gray(img[y:y+h, :])
        hsv_image = cv2.cvtColor(img[y:y+h, :], cv2.COLOR_RGB2HSV)
        # cv2.imshow('frame',img[y:y+h, :])
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
    def preprocess_imagerm(self, image): 
        # Load the image 
        # img = cv2.imread(image_path) 
        img = CvBridge().compressed_imgmsg_to_cv2(image, "bgr8")
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB) 
        x, y, w, h = 12, 12, 40, 40 #12 to 10, 40-44 
        img = cv2.resize(img, (64, 64)) 
        gray_image = color.rgb2gray(img[y:y+h, :]) 
        img = img[y:y+h, :] # keep all x 
        hsv_image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV) 
        # gray_image = cv2.resize(gray_image, (64, 64)) 
    
        sobelx = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0, ksize=3) 
        sobely = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1, ksize=3) 
        sobel_edges = np.hypot(sobelx, sobely) 
        edges_resized = 30 * sobel_edges.flatten() / 255 
    
        hue = hsv_image[:,:,0] 
        sat = hsv_image[:,:,1] 
        val = hsv_image[:,:,2] 
    
        #red mask 
        redHue = ((hue >0) & (hue < 25)) | ((hue> 170) & (hue<200)) 
        redSat = ((sat>150) & (sat <255)) 
        redVal = ((val>50) & (val <255)) 
        redMask = (redHue & redSat & redVal) 
        #green Mask  
        greenHue = (hue > 55) & (hue < 95) 
        greenSat = ((sat>100) & (sat <255)) 
        greenVal = ((val>30) & (val <150)) 
        greenMask = (greenHue & greenSat & greenVal) 
        #blue hue  
        blueHue = (hue > 110) & (hue < 130) 
        blueSat = ((sat>140) & (sat <235)) 
        blueVal = ((val>0) & (val <100)) 
        blueMask = (blueHue & blueSat & blueVal) 
    
        kernel = np.ones((3,3),np.uint8) 
        #Fills in the objects to help with contours  
        rm = (redMask* 255).astype(np.uint8) 
        bm = (blueMask* 255).astype(np.uint8) 
        gm = (greenMask* 255).astype(np.uint8) 
        rm = cv2.dilate(rm, kernel, iterations=5) 
        bm = cv2.dilate(bm, kernel, iterations=1) 
        gm = cv2.dilate(gm, kernel, iterations=2) 
    
        rm = rm.flatten() / 255.0 
        gm = gm.flatten() / 255.0 
        bm = bm.flatten() / 255.0 
        
        hog_features = hog(gray_image, orientations=8, pixels_per_cell=(8, 8), 
                        cells_per_block=(2, 2), block_norm='L2', feature_vector=True) 
    
        hsv_image = cv2.GaussianBlur(hsv_image, (5, 5), 0) 
        hsv_image = hsv_image.flatten()   
        hsv_image = hsv_image / 255.0  # norm  
    
        features = np.hstack([hog_features, hsv_image, edges_resized, rm])  
         
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
