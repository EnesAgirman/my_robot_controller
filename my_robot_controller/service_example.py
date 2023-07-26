#!/usr/bin/python

from functools import partial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from rclpy.clock import Clock
from turtlesim.srv import SetPen


class ServiceExample(Node):
    
    def __init__(self):\
        
        super().__init__("service_example")
        self.mySub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.myPub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        
        self.previous_x = 0.0
        self.previous_y = 0.0
        
        self.get_logger().info("the turtle controller node has been succesfully created")



    def pose_callback(self, pose: Pose):
        cmd = Twist()
        
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 1.5
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        
        if self.previous_x <= 5.0  and pose.x > 5.0:
            self.previous_x = pose.x
            self.previous_y = pose.y
            self.set_pen_callback(255, 255, 255, 3, 0)
        elif self.previous_x >= 5.0  and pose.x < 5.0:
            self.previous_x = pose.x
            self.previous_y = pose.y
            self.set_pen_callback(0, 255, 0, 3, 0)
        self.myPub.publish(cmd)
        
    
    def set_pen_callback( self, r, g, b, width, off ):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("waiting for service")
        
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off
        
        
        future = client.call_async(request)
        future.add_done_callback(partial(self.future_callback))
        
    def future_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("the service has been called")
        except Exception as e:
            self.get_logger().error("service call failed %r" % (e,))
        
        

def main(args=None):
    rclpy.init(args=None)
    myNode = ServiceExample()
    rclpy.spin(myNode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()