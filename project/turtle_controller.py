#!/usr/bin/env python3

# Importing the ros2 python library
# rcl stands for ROS Client Libraries
import rclpy
# Importing the node library, which does something
from rclpy.node import Node
# Importing the Twist message type for publishing velocity
from geometry_msgs.msg import Twist
# Importing the Pose message type for pose subscription
from turtlesim.msg import Pose
# Importing the math module for pythagorean theorem
import math


class TurtleControllerNode(Node):
    def __init__(self):
        # Naming the node
        super().__init__('turtle_controller')
        self.target_x = 8.0
        self.target_y = 4.0

        # A variable for the pose
        self.pose_ = None
        # Creating a velocity publisher
        self.velocity_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # Creating a subscriber
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.subscriber_callback, 10)
        
        # Frequency of the publisher
        timer_period = 0.01

        # Creating the timer
        self.control_loop_timer_ = self.create_timer(timer_period, self.control_loop)

        # Let us write an intializing message
        self.get_logger().info('The turtle controller has now started')

    def subscriber_callback(self, hello):
        self.pose_ = hello

    def control_loop(self):
        if self.pose_ == None:
            return 

        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y

        distance = math.sqrt(dist_x ** 2 + dist_y ** 2)

        msg = Twist()

        if distance > 0.5:
            # First the position
            msg.linear.x = 2*distance

            # The orientation
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi

            msg.angular.z = 6*diff
        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0

        self.velocity_publisher_.publish(msg)


# Defining the main function
def main(args=None):
    # Initializing ROS
    rclpy.init(args=args)
    node = TurtleControllerNode()
    # Spinning the node
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
