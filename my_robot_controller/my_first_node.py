#!/usr/bin/python
import rclpy
from rclpy.node import Node

class MyNodeClass(Node):
    
    def __init__(self):
        
        self.counter_ = 0
        super().__init__("my_first_node")   # create a node with this name
        self.create_timer(1.0, self.timer_callback)
        
        
    def timer_callback(self):
        self.get_logger().info("Hi" + str(self.counter_) )
        self.counter_ += 1

def main(args=None):
    rclpy.init(args=args)   # initialize ROS2 communications
    
    #
    
    myNode = MyNodeClass()   # create a node
    
    rclpy.spin(myNode)   # keep program alive until a ROS2 'shutdown' event is detected
    
    #
    
    rclpy.shutdown()    # shutdown ROS2 communications

if __name__ == '__main__':
    main()