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
        self.initialState = 1
        self.currentState = self.initialState
        self.waypoints = np.array([(1.5,0), (1.5,1.4), (0,1.4)]) # waypoints
        self.waypoint_reached = False # waypoint flag
        self.i = 0 # waypoint counter
        self.x_tau = self.globalPos
        self.nearest_obj = Point()
        self.nearest_obj.x = 44.0
        self.nearest_obj.y = 44.0
        self.nearest_obj.z = 44.0

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
        self.nearest_obj = data
        
    def odom_callback(self, data):
        self.update_Odometry(data)

        globPos = np.array([self.globalPos.x, self.globalPos.y])
        x_o = np.array([self.globalPos.x + self.nearest_obj.x*np.cos(self.globalAng + self.nearest_obj.z), self.globalPos.y + self.nearest_obj.x*np.sin(self.globalAng + self.nearest_obj.z)])  # distance from nearest point (x,y), position of nearest obstacle
        x_g =  self.waypoints[self.i] # position of goal (x,y)
        u_ao = globPos - x_o
        u_fwc = self.get_orthogonalc(u_ao) # vector pointing in the clockwise direction perpendicular to u_ao
        u_fwcc = self.get_orthogonalcc(u_ao) # vector pointing in the counterclockwise direction perpendicular to u_ao
        u_gtg = [(x_g[0] - self.globalPos.x), (x_g[1] - self.globalPos.y)] # vector pointing to the goal
        d = 0.25 # distance buffer from object [m], was 0.35
        eps = 0.15 # fat guard, minimum buffer zone from obstacle [m], was 0.2
        
        if (np.linalg.norm(globPos - x_g) <= .025):    
            self.i += 1
            if self.i == 3:
                self.vel.linear.x = 0.0
                self.vel.angular.z = 1.0
            else:
                x_g = self.waypoints[self.i]


        # State 1 u_gtg
        if self.currentState == 1:
            theta_d = np.arctan2(u_gtg[1], u_gtg[0])
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            e_theta = theta_d - theta_curr
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            
            k_theta = 2

            self.vel.angular.z = float(k_theta*e_theta)
            dist = np.linalg.norm(globPos - x_g)
            k_linear = 0.74 #was 0.24
            self.vel.linear.x = float(k_linear*dist)
            self.x_tau = globPos
            if (np.linalg.norm(globPos - x_o) <= d and np.inner(u_gtg,u_fwc) > 0):
                self.currentState = 2
            elif (np.linalg.norm(globPos - x_o) <= d and np.inner(u_gtg,u_fwcc) > 0):
                self.currentState = 3

        # State 2 u_fwc
        elif self.currentState == 2:
            theta_d = np.arctan2(u_fwc[1], u_fwc[0])
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            e_theta = theta_d - theta_curr
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            print("e_theta", e_theta)
            k_theta = 2
            self.vel.angular.z = k_theta*e_theta
            if abs(e_theta) < 0.1:
                self.vel.linear.x = 0.19
            else:
                self.vel.linear.x = 0.0


            if (np.linalg.norm(globPos - x_o) < d - eps):
                self.currentState = 4
            elif ((np.inner(u_gtg,u_ao) > 0 and np.linalg.norm(globPos - x_g) < np.linalg.norm(self.x_tau - x_g)) or np.linalg.norm(globPos - x_o) >= d + eps):
                self.currentState = 1
            
        # State 3 u_fwcc
        elif self.currentState == 3:
            theta_d = np.arctan2(u_fwcc[1], u_fwcc[0])
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            e_theta = theta_d - theta_curr
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            k_theta = 2
            self.vel.angular.z = k_theta*e_theta
            if abs(e_theta) < 0.1:
                self.vel.linear.x = 0.19
            else:
                self.vel.linear.x = 0.0


            if (np.linalg.norm(globPos - x_o) < d - eps):
                self.currentState = 4
            elif ((np.inner(u_gtg,u_ao) > 0 and np.linalg.norm(globPos - x_g) < np.linalg.norm(self.x_tau - x_g)) or np.linalg.norm(globPos - x_o) >= d + eps):
                self.currentState = 1

        # State 4 u_ao
        elif self.currentState == 4:
            dist = np.linalg.norm(globPos - x_o)
            k_linear = 0.24
            self.vel.linear.x = -k_linear*dist


            self.x_tau = globPos
            if (np.linalg.norm(globPos - x_o) > d and np.inner(u_gtg,u_fwc) > 0):
                self.currentState = 2
            elif (np.linalg.norm(globPos - x_o) > d and np.inner(u_gtg,u_fwcc) > 0):
                self.currentState = 3


        if (self.vel.linear.x > 0.15):
                self.vel.linear.x = 0.15
        elif (self.vel.linear.x < -0.15):
            self.vel.linear.x = -0.15
        theta_lim = 1.5
        if (self.vel.angular.z > theta_lim):
            self.vel.angular.z = theta_lim
        elif (self.vel.angular.z < -theta_lim):
            self.vel.angular.z = -theta_lim

        if self.i == 3:
            self.vel.linear.x = 0.0
            self.vel.angular.z = 1.0
        print("vel", self.vel)
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

    def get_orthogonalc(self, vec):
        return [vec[1], -vec[0]]
    
    def get_orthogonalcc(self, vec):
        return [-vec[1], vec[0]]
         
     


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