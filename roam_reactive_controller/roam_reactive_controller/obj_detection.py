#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import math

class ObjDetection(Node):
    def __init__(self):
        super().__init__("obj_detector")
        
        # the slice of the sensor plane which will be sensed by the bot
        self.declare_parameter("forward_angle", 120)
        
        # various distances and speeds for the robot
        self.declare_parameter("stop_dist", 0.35)
        self.declare_parameter("slow_dist", 0.50)
        self.declare_parameter("resume_dist", 0.65)
        self.declare_parameter("cruise_spd", 0.4)
        self.declare_parameter("slow_spd", 0.2)
        self.declare_parameter("rotate_spd", 0.5)
        
        self.forward_angle = float(self.get_parameter("forward_angle").value)
        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.slow_dist = float(self.get_parameter("slow_dist").value)
        self.resume_dist = float(self.get_parameter("resume_dist").value)
        self.cruise_spd = float(self.get_parameter("cruise_spd").value)
        self.slow_spd = float(self.get_parameter("slow_spd").value)
        self.rotate_spd = float(self.get_parameter("rotate_spd").value)
        
        self.subscriber_ = self.create_subscription(LaserScan, "scan", 
                                                    self.get_current_scan, qos_profile_sensor_data)
        self.publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.blocked = False
        self.turn_dir = 1.0
        
    def get_forward_indices(self, msg: LaserScan):

        #getting the data from /scan
        angle_min = msg.angle_min
        angle_inc = msg.angle_increment
      
        #calculating start and end indices for sector array  
        half_rad = math.radians(self.forward_angle / 2.0)
        start_angle = -half_rad
        end_angle = half_rad
        start_idx = int(round((start_angle - angle_min) / angle_inc))
        end_idx = int(round((end_angle - angle_min) / angle_inc))
        n = len(msg.ranges)        
        start_idx = max(0, min(n-1, start_idx))
        end_idx = max(0, min(n-1, end_idx))
        
        # switching the indices if the start index > end index
        if end_idx < start_idx:
            start_idx, end_idx = end_idx, start_idx
        return start_idx, end_idx

    def get_current_scan(self, msg: LaserScan):
        twist = Twist()
        
        # failsafe for empty data from /scan
        try:
            if not msg.ranges:
                self.get_logger().warn("Empty scans received!?")
                twist.linear.x = 0.0
                self.publisher_.publish(twist)
                return
            
            # filtering the received data for NaN and -INF and replacing with INF    
            ranges_arr = np.array(msg.ranges)
            ranges_arr = np.nan_to_num(ranges_arr, nan=np.inf, posinf=np.inf, neginf=np.inf)
            ranges_arr[(ranges_arr < msg.range_min) | (ranges_arr > msg.range_max)] = np.inf

            start_idx, end_idx = self.get_forward_indices(msg)
            sector_ = ranges_arr[start_idx:end_idx+1]
            
            # cutting the slice in two halves
            n = len(sector_) // 2
            left_half = sector_[:n]
            right_half = sector_[n:]
            
            if sector_.size == 0:
                min_dist = np.inf
                left_min = np.inf
                right_min = np.inf
            else:
                min_dist = np.min(sector_)
                left_min = np.min(left_half) if left_half.size else np.inf
                right_min = np.min(right_half) if right_half.size else np.inf
                
            if not self.blocked:
                # if the robot senses an object and is closer to it 
                # than the stopping distance then it changes 
                # the blocked status to true and stops
                if min_dist < self.stop_dist:
                    self.blocked = True
                    twist.linear.x = 0.0
                
                # the robot senses an object closer than the slowing
                # distance then it will reduce it's speed
                elif min_dist < self.slow_dist:
                    twist.linear.x = self.slow_spd
                    twist.angular.z = 0.0
                
                # if the robot senses nothing in front of it
                # then it hits the cruising speed  
                else:
                    twist.linear.x = self.cruise_spd
                    twist.angular.z = 0.0
                    
            else:
                # the robot continues to move at the cruising speed 
                # if the sensed object is further than the resuming distance and
                # the blocked status is set to false
                if min_dist > self.resume_dist:
                    self.blocked = False
                    twist.linear.x = self.cruise_spd
                    twist.angular.z = 0.0
                
                # the robot will rotate if the blocked status is true 
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = self.rotate_spd
                    
                # if the senses that the object is on the left and the minimum distance
                # in the right is more than the slowing distance
                # then the robot will turn the opposite way to avoid the object and vice versa
                if left_min < right_min and left_min < self.stop_dist and right_min > self.slow_dist:
                    twist.linear.x = 0.0
                    self.turn_dir = -1.0
                    twist.angular.z = self.turn_dir * self.rotate_spd                   
                
                elif right_min < left_min and left_min > self.slow_dist and right_min < self.stop_dist:
                    twist.linear.x = 0.0
                    self.turn_dir = 1.0
                    twist.angular.z = self.turn_dir * self.rotate_spd
                    
                # else the robot will reverse and turn to make itself unstuck
                else:
                    twist.linear.x = -self.slow_spd
                    twist.angular.z = self.rotate_spd                   
                    
            self.publisher_.publish(twist)
            
        except Exception as e:
            self.get_logger().warn(f"Exception: {e}")
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObjDetection()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()