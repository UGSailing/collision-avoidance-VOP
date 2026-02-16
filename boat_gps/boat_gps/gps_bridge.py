import rclpy
from rclpy.node import Node
from nmea_msgs.msg import Sentence
from rtcm_msgs.msg import Message
import serial

class GPSBridge(Node):
    def __init__(self):
        super().__init__('gps_bridge')
        
        # Configure Serial Port (Match your baudrate from Step 1)
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        
        # Publisher for NMEA (to driver & ntrip_client)
        self.nmea_pub = self.create_publisher(Sentence, 'nmea', 10)
        
        # Subscriber for RTCM (from ntrip_client)
        self.create_subscription(Message, 'rtcm', self.rtcm_callback, 10)
        
        # Timer to read serial loop
        self.create_timer(0.01, self.read_serial)
        self.get_logger().info("GPS Bridge Started. Reading/Writing to Serial...")

    def read_serial(self):
        if self.ser.in_waiting:
            try:
                line = self.ser.readline().decode('ascii', errors='ignore').strip()
                if line.startswith('$'):
                    msg = Sentence()
                    msg.header.stamp = self.get_clock().now().to_msg()
                    msg.header.frame_id = "gps_link"
                    msg.sentence = line
                    self.nmea_pub.publish(msg)
            except Exception as e:
                pass

    def rtcm_callback(self, msg):
        # Write RTCM data directly to the GPS hardware
        try:
            self.ser.write(bytes(msg.data))
        except Exception as e:
            self.get_logger().error(f"Failed to write RTCM: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GPSBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()