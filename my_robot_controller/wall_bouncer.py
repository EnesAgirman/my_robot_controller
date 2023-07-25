#!/usr/bin/python

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class WallBouncer(Node):
    
    def __init__(self):
        super().__init__("wall_bouncer")
        self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("the wall bouncer node has been succesfully created")
        self.xTurn = 3.0
        self.yTurn = 3.0

    def pose_callback(self, pose:Pose):

        self.get_logger().info("( " + str(pose.y) + " , " + str(pose.y) + " )")

        cmd = Twist()

        if (pose.x>2.1) & (pose.x<9.1) & (pose.y>2.1) & (pose.y<9.1):
            cmd.linear.x = 10.0 * self.xTurn
            cmd.linear.y = 5.0 * self.yTurn
        elif (pose.x<2.1) | (pose.x>9.1):
            self.xTurn=self.xTurn*-1.0
            cmd.linear.x = 10.0 * self.xTurn
            cmd.linear.y = 5.0 * self.yTurn
        elif (pose.y<2.1) | (pose.y>9.1):
            self.yTurn=self.yTurn*-1.0
            cmd.linear.x = 10.0 * self.xTurn
            cmd.linear.y = 5.0 * self.yTurn
        else:
            self.yTurn=self.yTurn*-1.0
            self.xTurn=self.xTurn*-1.0
            cmd.linear.x = 10.0 * self.xTurn
            cmd.linear.y = 5.0 * self.yTurn
            

        self.cmd_vel_pub_.publish(cmd) 

def main(args=None):
    rclpy.init(args=None)
    myNode = WallBouncer()
    rclpy.spin(myNode)
    rclpy.shutdown()
    
if __name__ == "__main__":
    main()



