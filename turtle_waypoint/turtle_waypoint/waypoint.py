#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from std_srvs.srv import Empty
from enum import auto, Enum
from interfaces.srv import Waypoints
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class State(Enum):
    MOVING = auto()
    STOPPED = auto()

class Waypoint(Node):
    def __init__(self):
        super().__init__("waypoint")
        self.curr_state = State.STOPPED
        self.waypoints = []
        self.cmd_vel_pub = self.create_publisher(Twist, '/turtle1/cmd_vel',10)
        self.pose_subscribe = self.create_subscription(Pose, '/turtle1/pose', 
                                                       self.update_pose, 10)
        self.toggle = self.create_service(Empty, 'toggle', self.toggle_callback)
        self.load  = self.create_service(Waypoints, 'load', self.load_callback)
        print("hello")
    
    def load_callback(self, request, response):
        self.waypoints.append((request.x1, request.y1))
        self.waypoints.append((request.x2, request.y2))
        self.waypoints.append((request.x3, request.y3))
        response.dist = math.sqrt(math.pow(self.waypoints[0][0] - self.waypoints[-1][0], 2) 
                                      + math.pow(self.waypoints[0][1] - self.waypoints[-1][1], 2))
        return response


    def toggle_callback(self, request, response):
        if self.curr_state == State.STOPPED:
            self.curr_state = State.MOVING
            self.get_logger().info("MOVING")
        else:
            self.curr_state = State.STOPPED
            self.get_logger().info("STOPPED")
        
        return response

    def update_pose(self, data: Pose):
        velocity_message = Twist()
        x = data.x
        y = data.y
        yaw = data.theta
        
        if self.curr_state == State.MOVING:
            if self.waypoints:
                x = self.waypoints[0][0]
                y = self.waypoints[0][1]
            k_linear = 0.5
            distance = abs(math.sqrt((x - data.x)**2) + math.sqrt((y - data.y)**2))

            linear_speed = distance * k_linear

            k_angular = 4.0
            desired_angle_goal = math.atan2(y - data.y, x - data.x)
            angular_speed = (desired_angle_goal - yaw) * k_angular

            velocity_message.linear.x = linear_speed
            velocity_message.angular.z = angular_speed

            self.cmd_vel_pub.publish(velocity_message)

            if distance < 0.01 and self.waypoints:
                self.get_logger().info('HERE!')
                self.waypoints.pop(0)

def main():
    rclpy.init()
    node = Waypoint()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
