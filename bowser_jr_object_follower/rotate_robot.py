import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np

# linear max 0.22 
# right max -2.84 z
# left max 2.84 z
class rotate_robot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.centroid_subscriber = self.create_subscription(
				Point,
				'/object_centroid',
				self.rotate_callback,
                10)
        self.centroid_subscriber
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.rotate_right = Twist()
        self.rotate_right.angular.z = -1.0
        self.rotate_left = Twist()
        self.rotate_left.angular.z = 1.0
        self.stop = Twist()
        self.stop.angular.z = 0.0

    def rotate_callback(self, centroid):
        x = centroid.x
        z = centroid.z
        if z == 44: \
            self.vel_publisher.publish(self.stop)
        elif x > 190:
            self.vel_publisher.publish(self.rotate_right)
        elif x > 150:
            self.vel_publisher.publish(self.stop)
        else:
            self.vel_publisher.publish(self.rotate_left)

def main():
	rclpy.init() #init routine needed for ROS2.
	rotate_robot_node = rotate_robot() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(rotate_robot_node) # Trigger callback processing.
	#Clean up and shutdown.
	rotate_robot_node.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()
