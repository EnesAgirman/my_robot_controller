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
        
        self.previous_x = 5.0
        self.previous_y = 5.0
        
        self.xSpeed = 4.0
        self.ySpeed = 3.2
        
        self.xSwitch = 1.0
        self.ySwitch = 1.0
        
        self.get_logger().info("the turtle controller node has been succesfully created")



    def pose_callback(self, pose: Pose):
        cmd = Twist()

        # the corners of the walls
        if (pose.x > 9.0 and pose.x > self.previous_x and pose.y > 9.0 and pose.y > self.previous_y) or (pose.x < 1.0 and pose.x < self.previous_x and pose.y < 1.0 and pose.y < self.previous_y) or (pose.x < 1.0 and pose.x < self.previous_x and pose.y > 9.0 and pose.y > self.previous_y) or (pose.x > 9.0 and pose.x > self.previous_x and pose.y < 1.0 and pose.y < self.previous_y):
            self.previous_x = pose.x
            self.previous_y = pose.y
            
            self.xSwitch = self.xSwitch * -1
            self.ySwitch = self.ySwitch * -1
            
        if pose.x > 9.0 and pose.x > self.previous_x:
            self.get_logger().info("icerideyim")

            self.previous_x = pose.x
            self.previous_y = pose.y
            
            self.xSwitch = self.xSwitch * -1
            self.set_pen_callback(255, 0, 0, 3, 0)

        
        elif pose.x < 1.0 and pose.x < self.previous_x:
            self.previous_x = pose.x
            self.previous_y = pose.y
            
            self.xSwitch = self.xSwitch * -1
            self.set_pen_callback(0, 255, 0, 3, 0)

            
        if pose.y > 9.0 and pose.y > self.previous_y:
            self.previous_x = pose.x
            self.previous_y = pose.y

            self.ySwitch = self.ySwitch * -1
            self.set_pen_callback(0, 0, 255, 3, 0)

            
        elif pose.y < 1.0 and pose.y < self.previous_y:
            self.previous_x = pose.x
            self.previous_y = pose.y
            
            self.ySwitch = self.ySwitch * -1
            self.set_pen_callback(255, 255, 255, 3, 0)

            
        cmd.linear.x = self.xSpeed * self.xSwitch
        cmd.linear.y = self.ySpeed * self.ySwitch
        
        # uncomment this part for the turtle line to change 
        # color depending on what part of the screen it is on. 
        # On the right side, it is green and 
        # on the left size it is white
        # if self.previous_x <= 5.0  and pose.x > 5.0:
        #     self.previous_x = pose.x
        #     self.previous_y = pose.y
        #     self.set_pen_callback(255, 255, 255, 3, 0)
            
        # elif self.previous_x >= 5.0  and pose.x < 5.0:
        #     self.previous_x = pose.x
        #     self.previous_y = pose.y
        #     self.set_pen_callback(0, 255, 0, 3, 0)
            
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