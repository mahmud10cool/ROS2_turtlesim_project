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


class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__('turtle_controller')

    # What do we have to do first?
    # What is the function of this node?
    # To control the turtle
    # Which means it will need to publish things to the turtlesim node
    # So I have to create a publisher to the turtlesim node
    # But before that I need to find the name of the topic
    # The topic for the velocity is /turtle1/cmd_vel

    # Let us create a publisher that will publish to that topic
    # The arguments that go inside the create_publisher function are what?
    # The first argument will be the type of message that is published to the topic
    # What is that? Let us find out ...
    # It is a Twist type of message found in the geometry_msgs folder
    # The second argument will be the topic name 
    # The third argument is the queue size
    # What is the queue size?


        self.velocity_publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
    # Next we have to create a timer for the publisher
    # To determine how frequently it should publish
    # But what are the arguments that go into the create_timer function???
    # Two things: 1. The timer period, 2. A callback function

    # Let's set the time period to something small so the velocity is regularly published
    # Let us also see what happens if we publish very infrequently
    # Answer: It's not a good idea for this project
    # I'll leave it to be 0.1 for now unless I need to make it slower
        timer_period = 0.1

        self.velocity_timer_ = self.create_timer(timer_period, self.timer_callback)

    # Let us write an intializing message

        self.get_logger().info('The velocity publisher has initialized ...')

    # Now this node must also have a subscriber to know the position of the turtle at any given time
    # This can be known by subscribing to the turtle1/pose topic
    # It uses the pose type of message from the turtlesim library
    # But what are the arguments that go into a create_subscription function
    # Three things: 1. Type of message, 2. Topic name, 3. A callback function

        self.create_subscription(Pose, '/turtle1/pose', self.subscriber_callback, 10)

    def timer_callback(self):
        # In this function we will assign what to publish in the time period given
        # Twist is a message type with two vectors; one for linear and one for angular velocity
        vel = Twist()
        # For the linear velocity
        vel.linear.x = 1.0
        vel.linear.y = 0.0
        vel.linear.z = 0.0
        # For the angular velocity
        vel.angular.x = 0.0
        vel.angular.y = 0.0
        vel.angular.z = 1.0

        self.velocity_publisher_.publish(vel)

    def subscriber_callback(self, hello):
        pose = hello
        self.x_pose = pose.x
        self.y_pose = pose.y
        self.theta_pose = pose.theta


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
