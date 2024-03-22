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


class get_nearest_object(Node):
    def __init__(self):
        super().__init__('get_nearest_object')


        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			durability=QoSDurabilityPolicy.VOLATILE,
			depth=1
		)

        self.range_subscriber = self.create_subscription(
				LaserScan,
				'/scan',
				self.range_callback,
                image_qos_profile)
        self.range_subscriber
        
        self.obj_publisher = self.create_publisher(Point, '/nearest_obj', 10)
        self.obj = Point()


    def range_callback(self, scan):
        ranges = np.array(scan.ranges)
        min_index = np.nanargmin(ranges)
    
        min_distance = ranges[min_index]
        
        angle_to_object = scan.angle_min + min_index * scan.angle_increment
   
        self.obj.x = float(min_distance)
        self.obj.z = float(angle_to_object)
        self.obj_publisher.publish(self.obj)
        #print(self.obj)                

        

def main():
	rclpy.init() #init routine needed for ROS2.
	get_nearest_object_node = get_nearest_object() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(get_nearest_object_node) # Trigger callback processing.
	#Clean up and shutdown.
	get_nearest_object_node.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()
