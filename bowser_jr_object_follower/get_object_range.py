import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

# linear max 0.22 
# right max -2.84 z
# left max 2.84 z

# angle_min: 0.9655348062515259
# angle_max: 6.15871000289917
# angle_increment: 0.02689322456717491
# time_increment: 0.0005260398611426353
# scan_time: 0.10158013552427292
# range_min: 0.0
# range_max: 100.0
# Camera FOV = 62.2 deg, 320 pixels wide


class get_object_range(Node):
    def __init__(self):
        super().__init__('get_object_range')


        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			durability=QoSDurabilityPolicy.VOLATILE,
			depth=1
		)

        self.centroid_subscriber = self.create_subscription(
				Point,
				'/object_centroid',
				self.centroid_callback,
                10)
        self.centroid_subscriber
        self.range_subscriber = self.create_subscription(
				LaserScan,
				'/scan',
				self.range_callback,
                image_qos_profile)
        self.centroid_subscriber

        self.ang_pos_cam = None
        
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rotate_right = Twist()
        self.rotate_right.angular.z = -1.0
        self.rotate_left = Twist()
        self.rotate_left.angular.z = 1.0
        self.stop = Twist()
        self.stop.angular.z = 0.0

    def centroid_callback(self, centroid):
        if centroid.z == 44:
            self.ang_pos_cam = None
        else: 
            x = centroid.x - 160 # make the middle of the frame
            self.ang_pos_cam = (62.2/320) * x * (np.pi/180) # ang pos in radians, left is (-), right is (+)
            #print("angle")
            #print(self.ang_pos_cam*(180/np.pi))
    def range_callback(self, laserScan):
        if self.ang_pos_cam == None:
            dist = None
        else: 
            ind = int((self.ang_pos_cam - laserScan.angle_min) / laserScan.angle_increment)
            dist = np.mean(laserScan.ranges[ind-25:ind+25])
            #print("dist")
            #print(dist)

        

def main():
	rclpy.init() #init routine needed for ROS2.
	get_object_range_node = get_object_range() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(get_object_range_node) # Trigger callback processing.
	#Clean up and shutdown.
	get_object_range_node.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()
