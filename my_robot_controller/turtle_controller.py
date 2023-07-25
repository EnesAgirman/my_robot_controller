#!/usr/bin/python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from rclpy.clock import Clock


class TurtleController(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.mySub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.myPub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("the turtle controller node has been succesfully created")
        
        self.speed = 3.0
        self.angle = 90.0
        self.angular_speed = self.speed*2*math.pi/360
        self.relative_angle = self.angle*2*math.pi/360
        
        self.isTurning = True
        self.xSpeed = 5.0
        self.ySpeed = 3.0



    def pose_callback(self, pose: Pose):
        cmd = Twist()
        
        
        if pose.x < 8.0 and pose.x > 1.0 and pose.y < 8.0 and pose.y > 1.0: 

            cmd.linear.x = self.xSpeed
            cmd.linear.y = self.ySpeed
            cmd.angular.z = 0.0
        elif self.isTurning == True:  
            self.get_logger().info("\nilk elif" + str(self.isTurning))
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            cmd.angular.z = 5.0
            self.create_timer(0.4, self.timer_callback)

            # current_angle = 0
            # current_angle < self.relative_angle
            # while(self.isTurning):
            #     t1 =  Clock().now()
            #     current_angle = self.angular_speed*(t1-t0)

        else:
            self.get_logger().info("\nelse: " + str(self.isTurning))
            self.get_logger().info(str(pose.x) + " " + str(pose.y))
            cmd.linear.x = self.xSpeed
            cmd.linear.y = self.ySpeed
            cmd.angular.z = 0.0
            if pose.x > 8.0 and pose.x < 1.0 and pose.y > 8.0 and pose.y < 1.0:
                self.isTurning = True
                self.get_logger().info("\nnested if in the else: " + str(self.isTurning))
            """ if pose.x > 8.0 and pose.x < 1.0 and pose.y > 8.0 and pose.y < 1.0:
                cmd.linear.x = 0.0
                cmd.linear.y = 0.0
                cmd.angular.z = 5.0
                self.create_timer(0.4, self.timer_callback2) """
        
            
        self.myPub.publish(cmd)
    
    
    def timer_callback(self):
        self.isTurning = False
        self.get_logger().info("\ntimer callback: " + str(self.isTurning))
   
        
        
        
        

def main(args=None):
    rclpy.init(args=None)
    myNode = TurtleController()
    rclpy.spin(myNode)
    rclpy.shutdown()


if __name__ == "__main__":
    main()