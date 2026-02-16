from launch import LaunchDescription
from launch_ros.actions import Node
import os
from dotenv import load_dotenv

def generate_launch_description():
    load_dotenv()  # reads variables from .env into os.environ

    flepos_username = os.getenv("FLEPOS_USERNAME")
    if not flepos_username:
        raise RuntimeError("Missing FLEPOS_USERNAME. Set it in your environment or .env file.")

    flepos_password = os.getenv("FLEPOS_PASSWORD")
    if not flepos_password:
        raise RuntimeError("Missing FLEPOS_PASSWORD. Set it in your environment or .env file.")

    return LaunchDescription([
        # 1. Start the Serial Bridge (Handles Hardware I/O)
        Node(
            package='boat_gps',
            executable='gps_bridge',  # The python script above
            name='gps_bridge'
        ),

        # 2. Start the NMEA Parser (Converts NMEA -> NavSatFix/Heading)
        Node(
            package='nmea_navsat_driver',
            executable='nmea_topic_driver', # Note: Using topic_driver, not serial_driver
            name='nmea_parser',
            remappings=[('nmea_sentence', 'nmea')]
        ),

        # 3. Start the NTRIP Client (FLEPOS connection)
        Node(
        package='ntrip_client',
        executable='ntrip_ros.py', 
        name='ntrip_client',
        parameters=[{
            'host': '3.64.78.173',     # FLEPOS address
            'port': 2101,                  # Standard port
            'mountpoint': 'FLEPOSVRS32GREC',     # Check FLEPOS for exact mountpoint
            'username': flepos_username,   # Your FLEPOS user
            'password': flepos_password,   # Your FLEPOS pass
            'rtcm_message_package': 'rtcm_msgs'
        }],
        # Remap to ensure it gets the GGA data for VRS
        remappings=[('nmea', 'nmea')] 
        ),

        # 4. Start the GPS Logger (Logs to CSV & Visualizes Path)
        Node(
            package='boat_gps',
            executable='gps_logger',
            name='gps_logger',
            parameters=[{'log_file': os.path.expanduser('~/gps_log.csv')}]
        )
    ])