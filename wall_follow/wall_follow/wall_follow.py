import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('wall_follow')
        self.speed = 0
        self.prev_error = 0.0
        self.integral = 0.0

        self.kp = 0.0
        self.ki = 0.0
        self.kd = 0.0
        

        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.0)
        self.declare_parameter('kd', 0.1)
        self.timer = self.create_timer(1, self.timer_callback)
        self.publisher_ = self.create_publisher(AckermannDriveStamped,'/drive',10)
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription = self.create_subscription(Odometry,'/odom',self.odom_callback,10)
        self.subscription = self.create_subscription(AckermannDriveStamped,'/drive', self.dummy_callback,10)

    def timer_callback(self):
        self.kp = self.get_parameter('kp').get_parameter_value().double_value
        self.ki = self.get_parameter('ki').get_parameter_value().double_value
        self.kd = self.get_parameter('kd').get_parameter_value().double_value

        #self.get_logger().info(f'Kp is: {self.kp}, Ki is: {self.ki}, Kd is: {self.kd}')

        param_kp = rclpy.parameter.Parameter('kp', rclpy.Parameter.Type.DOUBLE, self.kp)
        param_ki = rclpy.parameter.Parameter('ki', rclpy.Parameter.Type.DOUBLE, self.ki)
        param_kd = rclpy.parameter.Parameter('kd', rclpy.Parameter.Type.DOUBLE, self.kd)
        all_new_parameters = [param_kp,param_ki,param_kp]
        self.set_parameters(all_new_parameters)

    def dummy_callback(self, msg):
        pass
        
    def odom_callback(self, odom_msg):
        # TODO: update current speed
        self.speed = odom_msg.twist.twist.linear.x

    def scan_callback(self, scan_msg):
        drive_result = AckermannDriveStamped()
        ranges = np.array(scan_msg.ranges)
        b_distance = ranges[179]  # 4 * 45 - 1
        a_distance = ranges [459] # 4 * (70+45) - 1
        theta = np.pi/2.57 #180 % 70
        alfa_radians= np.arctan((a_distance*np.cos(theta)-b_distance)/(a_distance*np.sin(theta)))
        alfa_degrees = np.degrees(alfa_radians)
        dt = b_distance * np.cos(alfa_radians)
        predict_distance = 1.5 # distance !!!
        dt_1 = dt + predict_distance * np.sin(alfa_radians)
        error = dt - dt_1 
        self.integral += error  # integral term
        derivative = error-self.prev_error # derivative term



        steering_angle = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.get_logger().info(f'steering angle value is: {steering_angle}')
        self.prev_error = error 
        #steering_angle = np.clip(steering_angle, -1.05, 1.05)
        if 0 <= abs(alfa_degrees) < 10:
            drive_result.drive.speed = 1.5
            
        elif 10 <= abs(alfa_degrees) < 20:
            drive_result.drive.speed = 1.0
            
        else:
            drive_result.drive.speed = 0.5
        drive_result.drive.steering_angle = steering_angle
        self.publisher_.publish(drive_result)
        #self.get_logger().info(f'Drive Speed: {drive_result.drive.speed}, Steering Angle: {steering_angle}')
        #self.get_logger().info(f'b distance is: {b_distance}, a_distance is: {a_distance}, alfa angle in radians: {alfa_radians}, alfa angle in degrees: {alfa_degrees}, predicted D distance: {dt_1}')



def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()





