#!/usr/bin/python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose



class PoseSubscriber(Node):
    
    def __init__(self):
        super().__init__("pose_subscriber")
        self.mySub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        
        self.get_logger().info("the pose subscriber node has been succesfully created")


    def pose_callback(self, msg):
        self.get_logger().info("( " + str(msg.x) + " , " + str(msg.y) + " )")

def main(args=None):
    rclpy.init(args=None)
    myNode = PoseSubscriber()
    rclpy.spin(myNode)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()