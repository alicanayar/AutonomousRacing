#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive # type: ignore
from visualization_msgs.msg import Marker, MarkerArray
from tf2_msgs.msg import TFMessage
import math

class PurePursuit(Node):
    def __init__(self):
        super().__init__('pure_pursuit1')
        self.speed = 0.0
        self.steering_angle = 0.0
        self.declare_parameter("wp_file_path", "/arc2024/ws/src/pp_waypoints3.csv")
        self.wp_file_path = self.get_parameter("wp_file_path").get_parameter_value().string_value

        self.publisher_ = self.create_publisher(AckermannDriveStamped,'/drive',10)
        self.subscription = self.create_subscription(MarkerArray,'/path',self.waypoint_callback,10)
        self.subscription = self.create_subscription(TFMessage,'/tf',self.tf_callback,10)
        self.subscription_= self.create_subscription(Odometry,"/ego_racecar/odom", self.odom_callback,10)
        self.visualize_pub_marker = self.create_publisher(Marker, "/goal_waypoint", 10)

        self.waypoints= np.genfromtxt(self.wp_file_path, delimiter="," )
        self.l = 2.0 # l= 2.0 ,  distances < 1.5 fine value
        self.prev_dist_x = 0.0
        self.prev_dist_y = 0.0
       
    def waypoint_callback(self,distance_msg):   
        pass



           
           

    def odom_callback(self,odom_msg):
        drive_result = AckermannDriveStamped()
        self.speed = odom_msg.twist.twist.linear.x
        self.current_x = odom_msg.pose.pose.position.x
        self.current_y = odom_msg.pose.pose.position.y
        distances = np.sqrt((self.waypoints[:,0]- self.current_x)**2 + (self.waypoints[:,1] - self.current_y)**2)
        distances[distances < 1.5 ] = np.inf
        
        min_distance = np.min(distances)
        min_index = np.argmin(distances)

        

        self.goal_waypoint_x = self.waypoints[min_index][0]
        self.goal_waypoint_y = self.waypoints[min_index][1]
            
        

        while True:
            if self.prev_dist_x == 0.0 and self.prev_dist_y == 0.0:
                self.prev_dist_x = self.goal_waypoint_x
                self.prev_dist_y = self.goal_waypoint_y
                break
            fark =  np.sqrt((self.goal_waypoint_y- self.prev_dist_y)**2 + (self.goal_waypoint_x-self.prev_dist_x)**2)
            if fark >= 1.0:
                distances[min_index] = np.inf
                min_index = np.argmin(distances)
                self.goal_waypoint_x = self.waypoints[min_index][0]
                self.goal_waypoint_y = self.waypoints[min_index][1]
            elif fark < 1.0:
                self.prev_dist_x = self.goal_waypoint_x
                self.prev_dist_y = self.goal_waypoint_y
                break


                

            
          
        

        self.get_logger().info(f'min distance:{min_distance}, min index: {min_index}')


        marker = Marker()
        marker.id = -2
        marker.header.frame_id = "map"
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = self.goal_waypoint_x
        marker.pose.position.y = self.goal_waypoint_y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        self.visualize_pub_marker.publish(marker)
        
        #y_distance = (self.current_x - self.goal_waypoint_x)* self.l / min_distance   # l/min_distance = y_distance/(current-goal)            
        #angle = (2 * y_distance ) / (self.l **2)
        x= odom_msg.pose.pose.orientation.x
        y= odom_msg.pose.pose.orientation.y
        z= odom_msg.pose.pose.orientation.z
        w= odom_msg.pose.pose.orientation.w
        
        heading_current = math.atan2(2.0*(w*z + x*y), 1.0 - 2.0* (y*y+ z*z))
        euclidean_dist = math.dist([ self.goal_waypoint_y,self.goal_waypoint_x],[self.current_y,self.current_x])
        lookahead_angle1 = math.atan2(self.goal_waypoint_y- self.current_y,self.goal_waypoint_x- self.current_x)
        self.steering_angle = np.clip(self.steering_angle, -1.05, 1.05)
        delta_y = euclidean_dist * np.sin(lookahead_angle1-heading_current)
        #self.get_logger().info(f"steering angle: {self.steering_angle},min index: {min_index},min distance:{min_distance}")
        self.steering_angle = (2* delta_y) / (self.l**2)
        self.steering_angle = np.clip(self.steering_angle, -1.05, 1.05)
        
        #self.get_logger().info(f"steering angle: {self.steering_angle},min index: {min_index},min distance:{min_distance}")
        
        

        if 0 <= abs(self.steering_angle) < 0.174:
            drive_result.drive.speed = 5.0
        elif 0.174 <= abs(self.steering_angle) < 0.349:
            drive_result.drive.speed = 2.5
        else:
            drive_result.drive.speed = 1.0

        drive_result.drive.steering_angle = self.steering_angle
        self.publisher_.publish(drive_result)
        return self.prev_dist_x, self.prev_dist_y



    def tf_callback(self,tf_msg):
        pass
        









def main(args=None):
    rclpy.init(args=args)
    pure_pursuit1 = PurePursuit()
    rclpy.spin(pure_pursuit1)
    pure_pursuit1.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
