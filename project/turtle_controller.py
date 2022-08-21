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
# Importing the newly created interface files
from interfaces_robot.msg import*
# New catch turtle service module
from interfaces_robot.srv import*
# Importing functools for partial
from functools import partial

class TurtleControllerNode(Node):
    def __init__(self):
        # Naming the node
        super().__init__('turtle_controller')

        # Making a parameter on whether to catch the closest turtle or not
        self.declare_parameter('catch_closest_turtle_first', True)

        self.turtle_to_catch_ = None

        # Variable to catch closest turtle first
        self.catch_closest_turtle_first_ = True

        # A variable for the pose
        self.pose_ = None
        # Creating a velocity publisher
        self.velocity_publisher_ = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10)

        # Creating a subscriber
        self.pose_subscriber_ = self.create_subscription(
            Pose, '/turtle1/pose', self.callback_turtle_pose, 10)
        
        # Frequency of the publisher
        timer_period = 0.01

        # Creating the timer
        self.control_loop_timer_ = self.create_timer(timer_period, self.control_loop)

        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)

        # Let us write an intializing message
        self.get_logger().info('The turtle controller has now started')

    
    # Callback function to get the data regarding the pose of the message
    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    def callback_alive_turtles(self, msg):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                self.turtle_to_catch_ = msg.turtles[0]
    
    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return 

        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y

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
            # Target reached
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.velocity_publisher_.publish(msg)

    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, 'catch_turtle')
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")
        
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_catch_turtle, turtle_name=turtle_name))
    
    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(
                    f"Turtle {str(turtle_name)} could not be caught")
        except Exception as e:
            self.get_logger().error(f"Service call failed {(e,)}")


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
