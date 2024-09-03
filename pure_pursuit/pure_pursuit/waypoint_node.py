#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rclpy.parameter
from nav_msgs.msg import Odometry
import math
import atexit
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd

class WaypointNode(Node):
    def __init__(self):
        super().__init__('waypoint_node')
        self.declare_parameter("wp_file_path", "/arc2024/ws/src/pp_waypoints3.csv")
        self.declare_parameter("odom_topic", "/ego_racecar/odom")
        self.declare_parameter("save_waypoint_threshold", 0.3)
        self.timer = self.create_timer(1, self.timer_callback)

        self.wp_file_path = self.get_parameter("wp_file_path").get_parameter_value().string_value
        self.odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self.save_waypoint_threshold = self.get_parameter("save_waypoint_threshold").get_parameter_value().double_value


        
        self.subscription_= self.create_subscription(Odometry,self.odom_topic, self.odom_callback,10)

        self.wp_file = open(self.wp_file_path, "w")
        self.prev_waypoint = [0.0,0.0]
        self.dist = 0.0
        
    def timer_callback(self):  
        param_wp_file_path = rclpy.parameter.Parameter("wp_file_path",rclpy.Parameter.Type.STRING, self.wp_file_path)
        param_odom_topic = rclpy.parameter.Parameter("odom_topic",rclpy.Parameter.Type.STRING, self.odom_topic)
        param_save_waypoint_threshold = rclpy.parameter.Parameter("save_waypoint_threshold", rclpy.Parameter.Type.DOUBLE, self.save_waypoint_threshold)
        all_new_parameters = [param_wp_file_path,param_odom_topic,param_save_waypoint_threshold]
        self.set_parameters(all_new_parameters)
    
        

        


        
    def odom_callback(self, odom_msg):

        dx = odom_msg.pose.pose.position.x- self.prev_waypoint[0]
        dy = odom_msg.pose.pose.position.y - self.prev_waypoint[1]
        self.dist = math.pow(dx, 2) + math.pow(dy, 2)


        if self.dist > self.save_waypoint_threshold:
            self.wp_file.write(f"{odom_msg.pose.pose.position.x}, {odom_msg.pose.pose.position.y}\n" )
            self.prev_waypoint[0] = odom_msg.pose.pose.position.x
            self.prev_waypoint[1] = odom_msg.pose.pose.position.y

            self.get_logger().info(f"prev_waypoint x:{self.prev_waypoint}")    
    
        
    def shutdown(self):
        self.wp_file.close()
        self.get_logger().info("End")
        





def main(args= None):
    rclpy.init(args=args)
    waypoint_node = WaypointNode()
    atexit.register(waypoint_node.shutdown)
    rclpy.spin(waypoint_node)


#def main(args=None):
#    rclpy.init(args=args)
#    waypoint_node = WaypointNode()
#    rclpy.spin(waypoint_node)
#    waypoint_node.destroy_node()
#    rclpy.shutdown()


if __name__ == '__main__':
    main()
