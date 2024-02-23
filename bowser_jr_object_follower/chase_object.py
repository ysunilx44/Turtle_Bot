import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
import numpy as np
import time

# linear max 0.22 
# right max -2.84 z
# left max 2.84 z
class chase_robot(Node):
    def __init__(self):
        super().__init__('chase_robot')
        self.ball_pos_subscriber = self.create_subscription(
				Twist,
				'/ball_pos',
				self.rotate_callback,
                10)
        self.ball_pos_subscriber
        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.t_prev = time.time()
        self.e_x_prev = 0
        self.e_theta_prev = 0
        self.eSum_x = 0
        self.eSum_theta = 0


    def chase_callback(self, pos):
        b_x = pos.linear.x
        b_theta = pos.angular.z
        e_x = 0.3 - b_x
        e_theta = 0 - b_theta
        vel = Twist()
        t_curr = time.time()
        dt = t_curr - self.t_prev
        eDot_x = (e_x - self.e_x_prev) / dt
        eDot_theta = (e_theta - self.e_theta_prev) / dt
        self.eSum_x += self.eSum_x*dt
        self.eSum_theta += self.eSum_theta*dt

        vel.linear.x = 2*e_x + 0.2*eDot_x + 0.1*self.eSum_x
        vel.angular.z = 4*e_theta + 0.2*eDot_theta + 0.1*self.eSum_theta
        self.vel_publisher.publish(vel)
        self.e_x_prev = e_x
        self.e_theta_prev = e_theta
        self.t_prev = t_curr


def main():
	rclpy.init() #init routine needed for ROS2.
	chase_robot_node = chase_robot() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(chase_robot_node) # Trigger callback processing.
	#Clean up and shutdown.
	chase_robot_node.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()
