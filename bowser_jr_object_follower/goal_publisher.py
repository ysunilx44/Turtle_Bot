import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        time.sleep(1)

    def publish_goal(self, goal):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = goal['position'][0]
        msg.pose.position.y = goal['position'][1]
        msg.pose.position.z = goal['position'][2]
        msg.pose.orientation.x = goal['orientation'][0]
        msg.pose.orientation.y = goal['orientation'][1]
        msg.pose.orientation.z = goal['orientation'][2]
        msg.pose.orientation.w = goal['orientation'][3]

        self.publisher_.publish(msg)
        print("Publishing: ", msg)


def main(args=None):
    rclpy.init(args=args)

    goal_publisher = GoalPublisher()

    goal_points = [
        #{'position': [4.58, -0.719, 0.0], 'orientation': [0.0, 0.0, 0.0, 0.0]},
        #{'position': [3.41, 0.909, 0.0], 'orientation': [0.0, 0.0, 0.0, 0.0]},
        {'position': [0.598, -0.886, 0.0], 'orientation': [0.0, 0.0, 0.0, 0.0]},
    ]

    for goal in goal_points:
        goal_publisher.publish_goal(goal)
        time.sleep(10)  

    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
