#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
import pandas as pd
import numpy as np

class MarkerPublisher(Node):
    def __init__(self):
        super().__init__('markers_publisher')
        
        self.declare_parameter("wp_file_path", "/arc2024/ws/src/pp_waypoints3.csv")

        self.wp_file_path = self.get_parameter("wp_file_path").get_parameter_value().string_value

        #self.df=pd.read_csv(self.wp_file_path, sep= "\t",header= None)
        #self.df[["x","y"]] = self.df[0].str.split(",",expand=True)
        #self.df = self.df.drop(0,axis=1)
        self.array = np.genfromtxt(self.wp_file_path, delimiter="," )


        distances = np.sqrt((self.array[:,0]- 5)**2 + (self.array[:,1]-3)**2)
        min_index = np.argmin(distances)
        self.closest_waypoint_x = self.array[min_index][0]
        self.closest_waypoint_y = self.array[min_index][1]


        self.get_logger().info(f'np:{min_index}, first x:{self.closest_waypoint_x},first y: {self.closest_waypoint_y}')
        #self.get_logger().info(f'csv:{self.df}, x:{self.df.iloc[0,0]}, y: {self.df.iloc[0,1]}')

        self.publisher_ = self.create_publisher(MarkerArray,'/path',10)

        
    def marker_callback(self):
        while rclpy.ok():
            self.mark_result= MarkerArray()
            for i in range(len(self.array)):
                marker = Marker()
                marker.id = i
                marker.ns = "path"
                marker.header.frame_id = "map"
                marker.type = Marker.SPHERE
                marker.action = Marker.ADD
                marker.pose.position.x = self.array[i][0]
                marker.pose.position.y = self.array[i][1]
                marker.pose.position.z = 0.0
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = 0.0
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = 0.2
                marker.scale.y = 0.2
                marker.scale.z = 0.2
                marker.color.a = 1.0
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0

          
                self.mark_result.markers.append(marker)
            self.publisher_.publish(self.mark_result)











def main(args=None):
    rclpy.init(args=args)
    markers_publisher = MarkerPublisher()
    markers_publisher.marker_callback()
    
    markers_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
