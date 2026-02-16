import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import csv
import math
import os
from datetime import datetime

class GPSLogger(Node):
    def __init__(self):
        super().__init__('gps_logger')
        
        # Parameters
        self.declare_parameter('log_file', 'gps_log.csv')
        self.log_file_path = self.get_parameter('log_file').get_parameter_value().string_value
        
        # Publisher for visualization
        self.path_pub = self.create_publisher(Path, 'gps_path', 10)
        
        # Subscriber
        self.create_subscription(NavSatFix, 'fix', self.gps_callback, 10)
        
        # State
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        self.origin_lat = None
        self.origin_lon = None
        
        # Open CSV file
        self.f = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.f)
        self.csv_writer.writerow(['timestamp', 'latitude', 'longitude', 'altitude'])
        
        self.get_logger().info(f"GPS Logger Started. Logging to {self.log_file_path}")

    def gps_callback(self, msg):
        # Log to CSV
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.csv_writer.writerow([timestamp, msg.latitude, msg.longitude, msg.altitude])
        self.f.flush() # Ensure data is written
        
        # Visualization (Simple flat earth projection)
        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"Set origin to Lat: {self.origin_lat}, Lon: {self.origin_lon}")
            
        x, y = self.latlon_to_xy(msg.latitude, msg.longitude)
        
        pose = PoseStamped()
        pose.header = msg.header
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = msg.altitude
        
        self.path_msg.header.stamp = msg.header.stamp
        self.path_msg.poses.append(pose)
        
        self.path_pub.publish(self.path_msg)

    def latlon_to_xy(self, lat, lon):
        # Radius of Earth in meters
        R = 6378137.0 
        
        d_lat = math.radians(lat - self.origin_lat)
        d_lon = math.radians(lon - self.origin_lon)
        
        # Simple equirectangular projection (sufficient for small areas/visualization)
        x = R * d_lon * math.cos(math.radians(self.origin_lat))
        y = R * d_lat
        
        return x, y

    def destroy_node(self):
        self.f.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
