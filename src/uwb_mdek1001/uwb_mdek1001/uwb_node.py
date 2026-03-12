#!/usr/bin/env python3
"""
MDEK1001 UWB - ROS2 Distance Data Publisher
Otonom araç için UWB mesafe verisi yayıncısı
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseWithCovarianceStamped
import serial
import threading
import re
import json
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class MDEK1001Node(Node):
    """
    MDEK1001 UWB modülünden seri port üzerinden veri okuyup
    ROS2 topic'lerine yayınlayan node.
    """

    def __init__(self):
        super().__init__('mdek1001_uwb_node')

        # --- Parametreler ---
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('frame_id', 'uwb_link')
        self.declare_parameter('publish_rate', 10.0)   # Hz
        self.declare_parameter('min_range', 0.05)      # metre
        self.declare_parameter('max_range', 60.0)      # metre

        self.serial_port   = self.get_parameter('serial_port').value
        self.baud_rate     = self.get_parameter('baud_rate').value
        self.frame_id      = self.get_parameter('frame_id').value
        self.publish_rate  = self.get_parameter('publish_rate').value
        self.min_range     = self.get_parameter('min_range').value
        self.max_range     = self.get_parameter('max_range').value

        # --- QoS ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # --- Publisher'lar ---
        self.range_pub  = self.create_publisher(Range, 'uwb/range', qos)
        self.ranges_pub = self.create_publisher(Float32MultiArray, 'uwb/all_ranges', qos)
        self.pose_pub   = self.create_publisher(PoseWithCovarianceStamped, 'uwb/pose', qos)
        self.raw_pub    = self.create_publisher(String, 'uwb/raw', 10)

        # --- Seri port ---
        self.ser = None
        self.anchor_distances: dict[str, float] = {}
        self.lock = threading.Lock()

        self._connect_serial()

        # --- Okuma thread'i ---
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial, daemon=True)
        self.read_thread.start()

        # --- Yayın timer'ı ---
        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self._publish_data)

        self.get_logger().info(
            f'MDEK1001 UWB Node başlatıldı  →  port={self.serial_port}, baud={self.baud_rate}'
        )

    # ------------------------------------------------------------------ #
    #  Seri Port
    # ------------------------------------------------------------------ #

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            # MDEK1001'i "shell" modundan çıkarıp sürekli veri moduna al
            self.ser.write(b'\r\n')
            import time; time.sleep(0.1)
            self.ser.write(b'lec\r\n')   # Location Engine Continuous
            self.get_logger().info('Seri port bağlantısı kuruldu.')
        except serial.SerialException as e:
            self.get_logger().error(f'Seri port açılamadı: {e}')
            self.ser = None

    def _read_serial(self):
        """Arka planda seri port'tan satır okur ve ayrıştırır."""
        while self.running:
            if self.ser is None or not self.ser.is_open:
                import time; time.sleep(1.0)
                self._connect_serial()
                continue
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    self._parse_line(line)
            except serial.SerialException as e:
                self.get_logger().warn(f'Okuma hatası: {e}')
                self.ser = None

    # ------------------------------------------------------------------ #
    #  Veri Ayrıştırma
    # ------------------------------------------------------------------ #

    def _parse_line(self, line: str):
        """
        MDEK1001 'lec' çıktı formatları:
          DIST,<n>,AN0,<id>,<dist_m>, ... [,POS,<x>,<y>,<z>,<qf>]
          les komutu için benzer format
        """
        # Ham veriyi yayınla
        msg = String(); msg.data = line
        self.raw_pub.publish(msg)

        # ---- DIST satırı ----
        if line.startswith('DIST'):
            self._parse_dist(line)
        # ---- POS satırı (ayrı geliyorsa) ----
        elif line.startswith('POS'):
            self._parse_pos(line)

    def _parse_dist(self, line: str):
        """
        Örnek: DIST,4,AN0,DW1234,1.23,AN1,DW5678,2.34,...
        """
        parts = line.split(',')
        if len(parts) < 4:
            return

        distances: dict[str, float] = {}

        i = 2   # AN0 buradan başlar
        while i + 2 < len(parts):
            tag   = parts[i].strip()       # AN0, AN1 …
            anchor_id = parts[i+1].strip() # DW1234
            try:
                dist = float(parts[i+2])
                distances[anchor_id] = dist
            except ValueError:
                pass
            # POS bloğuna geçildiyse dur
            if i + 3 < len(parts) and parts[i+3] == 'POS':
                # konum bloğunu da işle
                self._parse_pos(','.join(parts[i+3:]))
                break
            i += 3

        with self.lock:
            self.anchor_distances.update(distances)

        self.get_logger().debug(f'Mesafeler: {distances}')

    def _parse_pos(self, segment: str):
        """
        Örnek: POS,1.23,4.56,0.00,98
        (x, y, z metre cinsinden, qf = kalite faktörü)
        """
        parts = segment.split(',')
        if len(parts) < 4:
            return
        try:
            x  = float(parts[1])
            y  = float(parts[2])
            z  = float(parts[3])
            qf = int(parts[4]) if len(parts) > 4 else 0

            msg = PoseWithCovarianceStamped()
            msg.header.stamp    = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            msg.pose.pose.position.x = x
            msg.pose.pose.position.y = y
            msg.pose.pose.position.z = z
            msg.pose.pose.orientation.w = 1.0
            # Basit kovaryans (qf düştükçe büyür)
            cov_val = max(0.01, (100 - qf) / 100.0)
            msg.pose.covariance[0]  = cov_val
            msg.pose.covariance[7]  = cov_val
            msg.pose.covariance[14] = cov_val * 4

            self.pose_pub.publish(msg)
            self.get_logger().debug(f'Konum: x={x:.3f}, y={y:.3f}, z={z:.3f}, qf={qf}')
        except (ValueError, IndexError) as e:
            self.get_logger().warn(f'POS ayrıştırma hatası: {e}')

    # ------------------------------------------------------------------ #
    #  Yayın
    # ------------------------------------------------------------------ #

    def _publish_data(self):
        with self.lock:
            distances = dict(self.anchor_distances)

        if not distances:
            return

        now = self.get_clock().now().to_msg()

        # Tüm mesafeleri FloatArray olarak yayınla
        arr_msg = Float32MultiArray()
        arr_msg.data = list(distances.values())
        self.ranges_pub.publish(arr_msg)

        # Her anchor için ayrı Range mesajı
        for anchor_id, dist in distances.items():
            if not (self.min_range <= dist <= self.max_range):
                continue
            r = Range()
            r.header.stamp    = now
            r.header.frame_id = f'{self.frame_id}/{anchor_id}'
            r.radiation_type  = Range.INFRARED   # UWB için en yakın tip
            r.field_of_view   = 0.0              # UWB yönlü değil
            r.min_range       = self.min_range
            r.max_range       = self.max_range
            r.range           = dist
            self.range_pub.publish(r)

    # ------------------------------------------------------------------ #
    #  Temizlik
    # ------------------------------------------------------------------ #

    def destroy_node(self):
        self.running = False
        if self.ser and self.ser.is_open:
            self.ser.write(b'reset\r\n')
            self.ser.close()
        super().destroy_node()


# ====================================================================== #
def main(args=None):
    rclpy.init(args=args)
    node = MDEK1001Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Kapatılıyor…')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()