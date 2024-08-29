#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

import numpy as np
# TODO: include needed ROS msg type headers and libraries
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
#from brake_msg.msg import brake

class SafetyNode(Node):
    """
    The class that handles emergency braking.
    """
    def __init__(self):
        super().__init__('safety_node')
        """
        One publisher should publish to the /drive_out topic with a AckermannDriveStamped drive message.

        You should also subscribe to the /scan topic to get the LaserScan messages and
        the /odom topic to get the current speed of the vehicle,
        and to /drive_in to get the drive commands.

        The subscribers should use the provided odom_callback and scan_callback as callback methods

        NOTE that the x component of the linear velocity in odom is the speed
        """
        self.speed = 0
        self.condition = False
        # bigger numbers cause early warning
        self.ttc_th = 5
        # TODO: create ROS subscribers and publishers.
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'/drive',10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.subscription = self.create_subscription(AckermannDriveStamped,'/drive', self.dummy_callback,10)

    def dummy_callback(self, msg):
        pass
        


    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        # TODO: calculate TTC
        ranges = np.array(scan_msg.ranges)
        min_angle = scan_msg.angle_min
        max_angle = scan_msg.angle_max
        increment = scan_msg.angle_increment
        beams_angle = np.arange(min_angle,max_angle,increment)
        pre_range_rate = self.speed * np.cos(beams_angle)
        rr = np.maximum(pre_range_rate,0)
        range_rate= np.where(rr == 0, 0.1,rr)
        ttc= ranges/ range_rate
        min_ttc = np.min(ttc)
        if  min_ttc < self.ttc_th:
            if not self.condition:
                self.speed = 0
                brake_msg = AckermannDriveStamped()
                self.publisher_.publish(brake_msg)
                self.get_logger().info('EMERGENCY BREAKING: ' )
                self.get_logger().info(f'min_ttc : {min_ttc}, speed : {self.speed}, publishing brake_msg : {brake_msg}')
                self.condition = True
        else:
            self.condition = False

        
            
        # TODO: publish command to brake
        



def main(args=None):
    rclpy.init(args=args)
    safety_node = SafetyNode()
    rclpy.spin(safety_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    safety_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
