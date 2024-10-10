#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

# import the message type to use
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist

class Heartbeat(Node):
    def __init__(self) -> None:
		# initialize base class (must happen before everything else)
        super().__init__("heartbeat")
        
		# create publisher with: self.create_publisher(<msg type>, <topic>, <qos>)
        self.hb_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # create a timer with: self.create_timer(<second>, <callback>)
        self.hb_timer = self.create_timer(0.2, self.hb_callback)
        
        self.motor_sub = self.create_subscription(Bool, "/kill", self.health_callback, 10)


    def hb_callback(self) -> None:
        """
        Heartbeat callback triggered by the timer
        """
        # construct heartbeat message
        # msg = String()
        # msg.data = "sending constant control…"

        # # publish heartbeat counter
        # self.hb_pub.publish(msg)
        
        msg = Twist()  # 0 initialize everything by default
        msg.linear.x = 1.0  # set this to be the linear velocity
        msg.angular.z = 2.0 # set this to be the angular velocity
        self.hb_pub.publish(msg)

        print("sending constant control…")

    def health_callback(self, msg: Bool) -> None:
        """
        Sensor health callback triggered by subscription
        """
        if msg.data:
            self.hb_timer.cancel()
            msg = Twist()  # 0 initialize everything by default
            msg.linear.x = 0.0  # set this to be the linear velocity
            msg.angular.z = 0.0 # set this to be the angular velocity
            self.hb_pub.publish(msg)


if __name__ == "__main__":
    rclpy.init()        # initialize ROS2 context (must run before any other rclpy call)
    node = Heartbeat()  # instantiate the heartbeat node
    rclpy.spin(node)    # Use ROS2 built-in schedular for executing the node
    rclpy.shutdown()    # cleanly shutdown ROS2 context