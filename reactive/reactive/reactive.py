#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


class ReactiveNode(Node):
    def __init__(self):
        super().__init__('reactive_node')
        self.speed = 0
        self.steering_angle =  0
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'/drive',10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.subscription = self.create_subscription(AckermannDriveStamped,'/drive', self.dummy_callback,10)

    def dummy_callback(self, msg):
        pass
        
    def odom_callback(self, odom_msg):
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        drive_result = AckermannDriveStamped()
        ranges = np.array(scan_msg.ranges)
        #if np.any(ranges<= 4):
        nearest_range = min(ranges[179:899]) # sola çok iyi dönüyor eğer index yazılmazsa sağa ççok iyi .
        nearest_index = np.argmin(ranges[179:899])
        radius= 0.6
        tan_side = radius / nearest_range
        scan_radians = np.arctan(tan_side) 
        scan_degree = int(np.degrees(scan_radians) *4 )
        min_range = max(0, nearest_index - scan_degree) 
        max_range = min(len(ranges),nearest_index + scan_degree) 
        for i in range(min_range, max_range):
            ranges[i] = 0.0
        #ranges[min_range:max_range] = [0.0] * (max_range-min_range)

            #self.get_logger().info(f'sıfırlanacak sayı: {scan_degree}, maxrange: {max_range}, minrange:{min_range}')
            
            
        
        furthest_range = max(ranges)
        best_point_degrees = np.argmax(ranges[179:899])/4
        best_point_radians = np.radians(best_point_degrees)
        self.steering_angle = best_point_radians - 1.565
        #self.steering_angle = np.clip(self.steering_angle, -1.05, 1.05)

        if 0 <= abs(self.steering_angle) < 0.174:
            drive_result.drive.speed = 2.0
        elif 0.174 <= abs(self.steering_angle) < 0.349:
            drive_result.drive.speed = 1.0
        else:
            drive_result.drive.speed = 0.5

        #self.get_logger().info(f'best point:{furthest_range},point radian:{best_point_radians},steering angle: {self.steering_angle}')
        drive_result.drive.steering_angle = self.steering_angle
        self.publisher_.publish(drive_result)








def main(args=None):
    rclpy.init(args=args)
    reactive_node = ReactiveNode()
    rclpy.spin(reactive_node)
    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
