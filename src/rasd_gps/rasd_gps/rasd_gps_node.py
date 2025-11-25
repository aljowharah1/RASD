#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from sensor_msgs.msg import NavSatFix

import serial
from datetime import datetime


class RASDGPSNode(Node):
    def __init__(self):
        super().__init__('rasd_gps_node')

        # -----------------------------
        # PARAMETERS
        # -----------------------------
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 9600)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        print(f"[GPS] Opening serial {port} @ {baud}...")

        # -----------------------------
        # OPEN SERIAL PORT
        # -----------------------------
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baud,
                timeout=1.0
            )
            print(f"[GPS] ✅ GPS UART opened successfully on {port} @ {baud}")
        except Exception as e:
            print(f"[GPS] ❌ Failed to open GPS serial: {e}")
            raise

        # -----------------------------
        # PUBLISHERS
        # -----------------------------
        self.pub_raw = self.create_publisher(String, '/gps/raw_nmea', 10)
        self.pub_speed = self.create_publisher(Float32, '/gps/speed_mps', 10)
        self.pub_fix = self.create_publisher(NavSatFix, '/gps/fix', 10)

        # -----------------------------
        # TIMER
        # -----------------------------
        self.timer = self.create_timer(0.1, self.read_gps)

        self.last_speed_mps = 0.0
        self.last_lat = None
        self.last_lon = None

        print("[GPS] Node initialized — waiting for NMEA data...")


    # ============================================================
    #             READ FROM GPS SERIAL
    # ============================================================
    def read_gps(self):

        try:
            line = self.ser.readline().decode('ascii', errors='ignore').strip()
        except Exception as e:
            print(f"[GPS] Read error: {e}")
            return

        if not line:
            return

        # ALWAYS PRINT RAW INPUT for debugging
        print(f"[GPS RAW] {line}")

        # Publish raw NMEA
        self.pub_raw.publish(String(data=line))

        # Identify sentence type
        if line.startswith('$GPRMC'):
            self.handle_gprmc(line)
        elif line.startswith('$GPGGA'):
            self.handle_gpgga(line)


    # ============================================================
    #             PARSE SPEED (GPRMC)
    # ============================================================
    def handle_gprmc(self, sentence: str):
        parts = sentence.split(',')

        if len(parts) < 8:
            print("[GPS] GPRMC missing fields")
            return

        status = parts[2]  # A = valid, V = invalid

        if status != 'A':
            print("[GPS] GPRMC status invalid")
            return

        sog_knots = 0.0
        try:
            sog_knots = float(parts[7]) if parts[7] else 0.0
        except ValueError:
            print("[GPS] Invalid knots value in GPRMC")

        # knots → m/s
        speed_mps = sog_knots * 0.514444
        self.last_speed_mps = speed_mps

        # Publish
        self.pub_speed.publish(Float32(data=speed_mps))

        # Print
        print(f"[GPS SPEED] {speed_mps:.2f} m/s  ({sog_knots:.1f} kn)")


    # ============================================================
    #             PARSE POSITION (GPGGA)
    # ============================================================
    def handle_gpgga(self, sentence: str):
        parts = sentence.split(',')
        if len(parts) < 10:
            print("[GPS] GPGGA missing fields")
            return

        lat_raw = parts[2]
        lat_ns  = parts[3]
        lon_raw = parts[4]
        lon_ew  = parts[5]
        fix_quality = parts[6]

        if fix_quality == '0':
            print("[GPS] No fix yet (quality=0)")
            return

        if not lat_raw or not lon_raw:
            print("[GPS] No lat/lon data")
            return

        try:
            lat = self.nmea_to_decimal(lat_raw, lat_ns)
            lon = self.nmea_to_decimal(lon_raw, lon_ew)
        except Exception as e:
            print(f"[GPS] Bad lat/lon conversion: {e}")
            return

        self.last_lat = lat
        self.last_lon = lon

        # Publish FIX
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.pub_fix.publish(msg)

        # Print
        print(f"[GPS FIX] lat={lat:.6f}, lon={lon:.6f}")


    # Convert ddmm.mmmm → decimal degrees
    def nmea_to_decimal(self, val: str, hemi: str) -> float:
        if len(val) < 3:
            return 0.0

        dot = val.find('.')
        deg_len = dot - 2 if dot != -1 else 2

        deg = float(val[:deg_len])
        mins = float(val[deg_len:])

        dec = deg + mins / 60.0

        if hemi in ('S', 'W'):
            dec = -dec

        return dec


def main(args=None):
    rclpy.init(args=args)
    node = RASDGPSNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

