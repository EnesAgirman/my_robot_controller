#!/usr/bin/python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from rclpy.clock import Clock


class ServiceExample(Node):
    
    def __init__(self):\
        
        super().__init__("service_example")
        self.mySub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.myPub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("the turtle controller node has been succesfully created")



    def pose_callback(self, pose: Pose):
        cmd = Twist()
        
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.5
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        
        self.myPub.publish(cmd)
        
        

def main(args=None):
    rclpy.init(args=None)
    myNode = ServiceExample()
    rclpy.spin(myNode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()