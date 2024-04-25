import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import numpy as np
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
import statistics as st

class nav_maze(Node):
    def __init__(self):
        super().__init__('nav_maze')

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
				'/frontal_dist',
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

        self.state_subscriber = self.create_subscription(
				Point,
				'/state',
				self.state_callback,
                10
                )
        self.state_subscriber

        self.vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.vel = Twist()
        self.state_changed = False
        self.curr_state = 1
        self.theta_d = 0.0
        self.frontal_dist = 100.0
        self.state = None
        self.count = 0


    def state_callback(self, state):
        self.state = state.x
        self.state_count = state.y

    def obj_callback(self, data):
        self.frontal_dist = data.x
        
    def odom_callback(self, data):
        self.update_Odometry(data)

        globPos = np.array([self.globalPos.x, self.globalPos.y])
        frontal_dist_d = 0.525 # 0.5 to 0.55
        eps = 0.013

        print("curr state",self.curr_state)

        #State machine
        if self.curr_state == 1: #Go straight
            # theta_d = np.arctan2(u_gtg[1], u_gtg[0])
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            e_theta = self.theta_d - theta_curr
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            k_theta = 2
            self.vel.angular.z = float(k_theta*e_theta)

            dist = self.frontal_dist - frontal_dist_d
            k_linear = 0.74 #was 0.24
            self.vel.linear.x = float(k_linear*dist)
            # self.x_tau = globPos
            print("dist", dist)
            if dist <= eps:
                 self.curr_state = 2
                 self.state_changed = True
        elif self.curr_state == 2: #Identify sign
            self.vel.angular.z = 0.0
            self.vel.linear.x = 0.0
            img_num = 100
            
            # for x in range(img_num):    
            #     states[0][x] = int(self.state)
            if self.state_changed:
                self.states = np.zeros((1, img_num))
                self.count = 0
                self.state_changed = False
            if self.count < img_num:
                self.states[0][self.count] = int(self.state)
                self.count += 1
                print("img_count", self.count)
                print("state from camera", self.state)
                print("state from camera count", self.state_count)
            else: 
                print("states vec", self.states[0])
                print("st mode", st.mode(self.states[0]))
                self.curr_state = int(st.mode(self.states[0]))
                # self.states[0][-1] = 6
                
                self.state_changed = True
                print("state from camera", self.curr_state)
        elif self.curr_state == 3: #Turn left
            if self.state_changed:
                self.wrap_theta(np.pi/2)
                self.state_changed = False
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            print("theta curr wrapped", theta_curr)
            e_theta = self.theta_d - theta_curr
            print("e_theta", e_theta)
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            print("e_theta wrapped", e_theta)
            k_theta = 2
            self.vel.angular.z = float(k_theta*e_theta)

            if abs(e_theta) < eps:
                self.curr_state = 1
                self.state_changed = True

        elif self.curr_state == 4: #Turn right
            if self.state_changed:
                self.wrap_theta(-np.pi/2)
                self.state_changed = False
            print("theta_d", self.theta_d)
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            print("theta curr wrapped", theta_curr)
            e_theta = self.theta_d - theta_curr
            print("e_theta", e_theta)
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            print("e_theta wrapped", e_theta)
            k_theta = 2
            self.vel.angular.z = float(k_theta*e_theta)

            if abs(e_theta) < eps:
                self.curr_state = 1
                self.state_changed = True

        elif self.curr_state == 5: #Turn around
            if self.state_changed:
                self.wrap_theta(np.pi)
                self.state_changed = False
            theta_curr = self.globalAng
            theta_curr = np.arctan2(np.sin(theta_curr), np.cos(theta_curr))
            e_theta = self.theta_d - theta_curr
            e_theta = np.arctan2(np.sin(e_theta), np.cos(e_theta))
            k_theta = 2
            self.vel.angular.z = float(k_theta*e_theta)

            if abs(e_theta) < eps:
                self.curr_state = 1
                self.state_changed = True

        elif self.curr_state == 6: #Goal
            self.vel.angular.z = 0.0
            self.vel.linear.x = 0.0

        if (self.vel.linear.x > 0.15):
                self.vel.linear.x = 0.15
        elif (self.vel.linear.x < -0.15):
            self.vel.linear.x = -0.15
        theta_lim = 1.5
        if (self.vel.angular.z > theta_lim):
            self.vel.angular.z = theta_lim
        elif (self.vel.angular.z < -theta_lim):
            self.vel.angular.z = -theta_lim

        # print("vel", self.vel)
        self.vel_publisher.publish(self.vel)
            
    def wrap_theta(self, theta):
        self.theta_d += theta
        if self.theta_d > 2*np.pi:
            self.theta_d = self.theta_d - 2*np.pi
        elif self.theta_d < 0:
            self.theta_d = self.theta_d + 2*np.pi

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
        # print('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))
        # self.get_logger().info('Transformed global pose is x:{}, y:{}, a:{}'.format(self.globalPos.x,self.globalPos.y,self.globalAng))

def main():
	rclpy.init() #init routine needed for ROS2.
	nav_maze_node = nav_maze() #Create class object to be used.

	while rclpy.ok():
		rclpy.spin_once(nav_maze_node) # Trigger callback processing.
	#Clean up and shutdown.
	nav_maze_node.destroy_node()  
	rclpy.shutdown()

if __name__ == '__main__':
	main()