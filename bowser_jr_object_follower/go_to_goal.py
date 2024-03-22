import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

class go_to_goal(Node):
    def __init__(self):
        super().__init__('go_to_goal')

        self.Init = True
        self.Init_pos = Point()
        self.Init_pos.x = 0.0
        self.Init_pos.y = 0.0
        self.Init_ang = 0.0
        self.globalPos = Point()

        #Set up QoS Profiles for passing images over WiFi
        image_qos_profile = QoSProfile(
			reliability=QoSReliabilityPolicy.BEST_EFFORT,
			history=QoSHistoryPolicy.KEEP_LAST,
			durability=QoSDurabilityPolicy.VOLATILE,
			depth=1
		)

        self.obj_subscriber = self.create_subscription(
				Point,
				'/nearest_obj',
				self.obj_callback,
                10
                )
        self.obj_subscriber
        
        self.odom_subscriber = self.create_subscription(
				Odometry,
				'/odom',
				self.odom_callback,
                10
                )
        self.odom_subscriber

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel = Twist()


    def obj_callback(self, data):
        x = 1
        
    def odom_callback(self, data):
        self.update_Odometry(data)
        if self.globalPos.x < 1.5:
            self.vel.linear.x = 0.19 # Make sure your commands are below 0.2m/s linear and 1.5 rad/s angular
            self.vel_publisher.publish(self.vel)
        else:
            self.vel.linear.x = 0.0
            self.vel_publisher.publish(self.vel)
        
    def update_Odometry(self,Odom):
        position = Odom.pose.pose.position
        #Orientation uses the quaternion aprametrization.
        #To get the angular position along the z-axis, the following equation is required.
        q = Odom.pose.pose.orientation
        orientation = np.arctan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z))
        if self.Init:
        #The initial data is stored to by subtracted to all the other values as we want to start at position (0,0) and orientation 0
            self.Init = False
            self.Init_ang = orientation
            self.globalAng = self.Init_ang
            Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
            self.Init_pos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y
            self.Init_pos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y
            self.Init_pos.z = position.z
        Mrot = np.matrix([[np.cos(self.Init_ang), np.sin(self.Init_ang)],[-np.sin(self.Init_ang), np.cos(self.Init_ang)]])
        #We subtract the initial values
        self.globalPos.x = Mrot.item((0,0))*position.x + Mrot.item((0,1))*position.y - self.Init_pos.x
        self.globalPos.y = Mrot.item((1,0))*position.x + Mrot.item((1,1))*position.y - self.Init_pos.y
        self.globalAng = orientation - self.Init_ang
        self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))
                

def main():
	rclpy.init() #init routine needed for ROS2.
	go_to_goal_node = go_to_goal() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(go_to_goal_node) # Trigger callback processing.
	#Clean up and shutdown.
	go_to_goal_node.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()
