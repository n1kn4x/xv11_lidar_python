#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

PI = 3.14159265

def has_valid_crc(dataframe):
    # group the data by word, little-endian
    data_list = []
    for t in range(10):
        data_list.append(dataframe[2*t] + (dataframe[2*t+1] << 8))
    # compute the checksum on 32 bits
    chk32 = 0
    for d in data_list:
        chk32 = (chk32 << 1) + d
    # wrap around to fit into 15 bits
    chk32 = (chk32 & 0x7FFF) + (chk32 >> 15)
    # truncate to 15 bits
    chk32 = chk32 & 0x7FFF
    # check if the calculated checksum is equal to the last 2 bytes of the message
    crc = dataframe[20] + (dataframe[21] << 8)
    return crc == chk32

def extract_sweep(buffer):
    """
        Each dataframe has 22 bytes. The 0'th is always 0xFA.
        Each dataframe contains 4 sensor readings.
        The 1st byte of dataframe is the index byte, going 
        from 0xA0 (packet 0, readings 0 to 3)
        to 0xF9 (packet 89, readings 356 to 359).
        The 2nd and 3rd byte contain the speed, a two-byte information, little-endian.
        It represents the speed, in 64th of RPM (aka value in RPM represented
        in fixed point, with 6 bits used for the decimal part).
        The 6'th bit of byte 1 is a warning when the reported strength is greatly inferior to what is expected at this distance.
        Then the 4th to the 7th are a range (in mm) + intensity reading, also 8th - 11th, 12th-15th, 16th-19th.
        The last two bytes (20th,21st) are a CRC over the dataframe.
    """
    buf_len = len(buffer)
    scan = LaserScan()
    scan.angle_min = 0.0
    scan.angle_max = 2*PI
    scan.angle_increment = (2*PI)/360
    scan.ranges = [0.0] * 360
    scan.intensities = [0.0] * 360
    sum_motor_speed, num_good_dfs = 0.0, 0.0 # Needed to average over all motor speed readings
    last_df = 0 # Needed to later throw away old dataframes
    i = 0
    while (i < buf_len):
        # The first byte of a dataframe must be 0xFA
        if (buffer[i] == 0xFA):
            # If there are 22 bytes following, we process the dataframe
            if (i+22 <= buf_len):
                # CRC check, then process the data frame
                if (has_valid_crc(buffer[i:i+22])):
                    df_index = buffer[i+1] - 0xA0
                    sum_motor_speed += buffer[i+2] + (buffer[i+3] << 8)
                    num_good_dfs += 1
                    # Not used but fiy: rpm = rotation_speed / 64
                    # Process the 4 sensor readings in each dataframe
                    for j in range(4):
                        byte0, byte1, byte2, byte3 = buffer[i+4+4*j], buffer[i+5+4*j], buffer[i+6+4*j], buffer[i+7+4*j]
                        scan.ranges[df_index*4+j] = (byte0 + ((byte1 & 0x3F) << 8)) / 1000.0
                        scan.intensities[df_index*4+j] = byte2 + (byte3 << 8)
                    # Since the dataframe had a correct CRC, the next dataframe should start in 22 bytes
                    i += 22
                    last_df = i
                # CRC was not correct, search for next 0xFA
                else:
                    i += 1
                    last_df = i
            # We're at the end of the buffer, no next dataframe
            else:
                break
        # The byte is not 0xFA, continue to search
        else: 
            i += 1
    # Finally, set the time increment and return the laser scan message
    scan.time_increment = sum_motor_speed/num_good_dfs/1e8
    return last_df, scan

class XV11_Node(Node):
    def __init__(self):
        super().__init__("xv11_lidar")
        self.get_logger().info("Initializing XV11 Lidar Node.")
        self.publisher = self.create_publisher(LaserScan, "/scan", 10)
        self.declare_parameter("port", "/dev/ttyXV11")
        self.declare_parameter("frame_id", "xv11_lidar")
        self.declare_parameter("range_min", 0.06)
        self.declare_parameter("range_max", 13.0)

    def publish_scan(self, scan):
        self.publisher.publish(scan)


""" MAIN """
def main(args=None):
    rclpy.init(args=args)
    node = XV11_Node()
    port = node.get_parameter("port").value
    data_buffer = []

    # Open Serial port
    with serial.Serial(port, 115200, timeout=1) as lidar_dev:
        if lidar_dev.isOpen():
            node.get_logger().info("{} connected!".format(lidar_dev.port))
        else:
            node.get_logger().error("Couldn't connect to {}!".format(lidar_dev.port))
            return
        # Scan and publish
        while (rclpy.get_default_context().ok()):
            t1 = node.get_clock().now() # Needed to calculate the scan time
            data_buffer += list(lidar_dev.read(100))
            if (len(data_buffer) >= 2024): # The data buffer should now contain an entire 360 sweep
                last_index, scan = extract_sweep(data_buffer)
                t2 = node.get_clock().now()
                scan.header.stamp = t2.to_msg()
                scan.header.frame_id = node.get_parameter("frame_id").value
                scan.range_min = node.get_parameter("range_min").value
                scan.range_max = node.get_parameter("range_max").value
                scan.scan_time = (t2-t1).nanoseconds *1e-9
                node.publish_scan(scan)
                data_buffer = data_buffer[last_index:]
    rclpy.shutdown()

if __name__ == "__main__":
    main()