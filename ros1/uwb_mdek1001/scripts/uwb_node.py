#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
MDEK1001 UWB - ROS 1 (rospy) Distance/Position Publisher
Otonom araç için UWB mesafe ve konum verisi yayıncısı.

Python 2.7 (ROS Melodic) ve Python 3 (ROS Noetic) uyumludur.

Her ölçüm cihazdan geldiği anda, alındığı anın zaman damgasıyla BİR KEZ yayınlanır.
Eski veri tekrar yayınlanmaz; verinin tazeliğine tüketici (EKF / kontrolcü)
header.stamp ve /diagnostics üzerinden karar verir.

Yayınlanan topic'ler:
  uwb/pose       geometry_msgs/PoseWithCovarianceStamped  (tag'in hesapladığı konum)
  uwb/range      sensor_msgs/Range  (anchor başına; frame_id = <frame_id>/<anchor_id>)
  uwb/all_ranges std_msgs/Float32MultiArray  (satırdaki mesafeler, anchor ID sırasıyla)
  uwb/raw        std_msgs/String  (cihazdan gelen ham satır)
  /diagnostics   diagnostic_msgs/DiagnosticArray  ("UWB mesafe" ve "UWB konum" durumları)
"""

import sys
import threading
import time

import rospy
import serial
from std_msgs.msg import Float32MultiArray, String
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseWithCovarianceStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from lec_parser import parse_lec_line

PY2 = sys.version_info[0] == 2
monotonic = getattr(time, 'monotonic', time.time)   # time.monotonic Python 2.7'de yok


class MDEK1001Node(object):
    """
    MDEK1001 UWB modülünden seri port üzerinden veri okuyup
    ROS topic'lerine yayınlayan node.
    """

    def __init__(self):
        # --- Parametreler (~ = node'a özel) ---
        self.serial_port          = rospy.get_param('~serial_port', '/dev/ttyACM0')
        self.baud_rate            = int(rospy.get_param('~baud_rate', 115200))
        self.frame_id             = rospy.get_param('~frame_id', 'uwb_link')
        self.pose_frame_id        = rospy.get_param('~pose_frame_id', 'map')
        self.min_range            = float(rospy.get_param('~min_range', 0.05))         # metre
        self.max_range            = float(rospy.get_param('~max_range', 60.0))         # metre
        self.latency_offset       = rospy.Duration(float(rospy.get_param('~latency_offset', 0.0)))  # sn
        self.orientation_variance = float(rospy.get_param('~orientation_variance', 1.0e6))  # UWB yönelim ölçmez
        self.z_variance           = float(rospy.get_param('~z_variance', 1.0e6))       # m², z güvenilmez
        self.pos_std_base         = float(rospy.get_param('~pos_std_base', 0.05))      # m, qf=100 iken std
        self.pos_std_max          = float(rospy.get_param('~pos_std_max', 2.0))        # m, üst sınır
        self.status_rate          = float(rospy.get_param('~status_rate', 10.0))       # Hz, /diagnostics
        self.warn_age             = float(rospy.get_param('~warn_age', 0.3))           # sn
        self.error_age            = float(rospy.get_param('~error_age', 1.0))          # sn
        self.data_timeout         = float(rospy.get_param('~data_timeout', 3.0))       # sn, sonra 'lec' yeniden

        # --- Publisher'lar ---
        self.range_pub  = rospy.Publisher('uwb/range', Range, queue_size=10)
        self.ranges_pub = rospy.Publisher('uwb/all_ranges', Float32MultiArray, queue_size=10)
        self.pose_pub   = rospy.Publisher('uwb/pose', PoseWithCovarianceStamped, queue_size=10)
        self.raw_pub    = rospy.Publisher('uwb/raw', String, queue_size=10)
        self.diag_pub   = rospy.Publisher('/diagnostics', DiagnosticArray, queue_size=10)

        # --- Seri port ---
        self.ser = None
        self.last_line_time = 0.0

        # --- Sağlık istatistikleri (okuma thread'i yazar, status timer'ı okur) ---
        self.lock = threading.Lock()
        self.last_meas_time = None     # monotonic sn, son geçerli ölçüm
        self.last_pose_time = None     # monotonic sn, son geçerli konum
        self.last_anchor_count = 0
        self.last_qf = 0
        self.meas_interval = None      # sn, ölçüm aralığının üstel ortalaması
        self.meas_count = 0

        # --- Okuma thread'i (bağlantıyı da kendisi kurar) ---
        self.running = True
        self.read_thread = threading.Thread(target=self._read_serial)
        self.read_thread.daemon = True   # Python 2.7: Thread(daemon=...) parametresi yok
        self.read_thread.start()

        # --- Sağlık yayını: veri kesilse bile sabit hızda yayınlanır ---
        self.status_timer = rospy.Timer(rospy.Duration(1.0 / self.status_rate), self._publish_status)

        rospy.on_shutdown(self.shutdown)
        rospy.loginfo('MDEK1001 UWB Node başlatıldı  ->  port={}, baud={}'.format(
            self.serial_port, self.baud_rate))

    # ------------------------------------------------------------------ #
    #  Seri Port
    # ------------------------------------------------------------------ #

    def _connect_serial(self):
        kwargs = dict(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=1.0,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
        )
        try:
            try:
                # aynı portu başka bir süreç (ikinci node vb.) açamasın
                self.ser = serial.Serial(exclusive=True, **kwargs)
            except TypeError:
                self.ser = serial.Serial(**kwargs)   # pyserial < 3.3: exclusive desteklenmiyor
        except (serial.SerialException, OSError) as e:
            rospy.logerr_throttle(10.0, 'Seri port açılamadı: {}'.format(e))
            self.ser = None
            return False

        rospy.loginfo('Seri port bağlantısı kuruldu.')
        self._start_lec()
        return True

    def _start_lec(self):
        """
        Cihazı shell moduna alıp sürekli konum/mesafe çıktısını ('lec') başlatır.

        Açılışta cihaz binary (TLV) API modundadır; shell'e geçmek için
        1 sn içinde iki kez Enter (CR) gerekir. Cihaz zaten shell'deyse
        bu Enter'lar sadece boş satır olur ya da çalışan çıktıyı durdurur,
        ardından gelen 'lec' akışı yeniden başlatır.
        """
        self.ser.reset_input_buffer()
        self.ser.write(b'\r\r')
        time.sleep(1.0)                 # shell banner'ı ve 'dwm>' prompt'u için
        self.ser.reset_input_buffer()
        self.ser.write(b'lec\r')        # sadece CR: LF ikinci bir Enter sayılıp akışı durdurabilir
        self.last_line_time = monotonic()
        rospy.loginfo("'lec' komutu gönderildi, veri bekleniyor...")

    def _close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
        self.ser = None

    def _read_serial(self):
        """Arka planda seri port'tan satır okur ve ayrıştırır."""
        while self.running and not rospy.is_shutdown():
            if self.ser is None or not self.ser.is_open:
                if not self._connect_serial():
                    time.sleep(1.0)
                continue
            try:
                raw = self.ser.readline()
                # Zaman damgası satır okunur okunmaz alınır; ayrıştırma süresi ölçüme eklenmez
                stamp = rospy.Time.now() - self.latency_offset
                # Cihaz çıktısı ASCII; gürültü baytları atılır. Python 2'de native str'e çevrilir.
                line = raw.decode('ascii', 'ignore').strip()
                if PY2:
                    line = line.encode('ascii')
                if line:
                    self.last_line_time = monotonic()
                    self._handle_line(line, stamp)
                elif monotonic() - self.last_line_time > self.data_timeout:
                    rospy.logwarn(
                        '{:.0f} sn boyunca veri gelmedi, lec yeniden başlatılıyor. '
                        '(Cihaz tag modunda mı? Port başka bir programda açık mı?)'.format(self.data_timeout))
                    self._start_lec()
            except (serial.SerialException, OSError) as e:
                if self.running:
                    rospy.logwarn('Okuma hatası: {}'.format(e))
                self._close_serial()

    # ------------------------------------------------------------------ #
    #  Veri İşleme ve Yayın
    # ------------------------------------------------------------------ #

    def _handle_line(self, line, stamp):
        # Ham veriyi yayınla
        self.raw_pub.publish(String(data=line))

        data = parse_lec_line(line)
        if data is None:
            rospy.logdebug('Tanınmayan satır: {!r}'.format(line))
            return
        if not data.distances and data.position is None:
            return   # ör. 'DIST,0' -> hiç anchor görülmüyor; ölçüm sayılmaz

        if data.distances:
            self._publish_ranges(data, stamp)
        if data.position is not None:
            self._publish_pose(data, stamp)
        self._update_stats(data)

    def _publish_ranges(self, data, stamp):
        items = sorted(data.distances.items())   # sabit sıra: anchor ID

        # Bu satırdaki tüm mesafeler (anchor ID sırasıyla)
        self.ranges_pub.publish(Float32MultiArray(data=[d for _, d in items]))

        # Her anchor için ayrı Range mesajı; anchor ID frame_id içinde
        for anchor_id, dist in items:
            if not (self.min_range <= dist <= self.max_range):
                continue
            r = Range()
            r.header.stamp    = stamp
            r.header.frame_id = '{}/{}'.format(self.frame_id, anchor_id)
            r.radiation_type  = Range.INFRARED   # UWB için en yakın tip
            r.field_of_view   = 0.0              # UWB yönlü değil
            r.min_range       = self.min_range
            r.max_range       = self.max_range
            r.range           = dist
            self.range_pub.publish(r)

        rospy.logdebug('Mesafeler: {}'.format(dict(items)))

    def _publish_pose(self, data, stamp):
        pos = data.position
        msg = PoseWithCovarianceStamped()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.pose_frame_id
        msg.pose.pose.position.x = pos.x
        msg.pose.pose.position.y = pos.y
        msg.pose.pose.position.z = pos.z
        msg.pose.pose.orientation.w = 1.0

        # Yatay konum varyansı (m²): std = pos_std_base * 100 / qf
        # (qf=80 -> 6.3 cm; masa testinde ölçülen yatay hata ~6.6 cm)
        std = min(self.pos_std_max, self.pos_std_base * 100.0 / max(pos.qf, 1))
        cov = [0.0] * 36
        cov[0]  = std * std
        cov[7]  = std * std
        cov[14] = self.z_variance
        # Yönelim ölçülmüyor: EKF bu eksenlere güvenmesin
        cov[21] = self.orientation_variance
        cov[28] = self.orientation_variance
        cov[35] = self.orientation_variance
        msg.pose.covariance = cov

        self.pose_pub.publish(msg)
        rospy.logdebug('Konum: x={:.3f}, y={:.3f}, z={:.3f}, qf={}'.format(pos.x, pos.y, pos.z, pos.qf))

    def _update_stats(self, data):
        now = monotonic()
        with self.lock:
            if self.last_meas_time is not None:
                dt = now - self.last_meas_time
                self.meas_interval = dt if self.meas_interval is None \
                    else 0.9 * self.meas_interval + 0.1 * dt
            else:
                rospy.loginfo('İlk ölçüm alındı: {} {}'.format(data.distances, data.position))
            self.last_meas_time = now
            self.meas_count += 1
            if data.distances:
                self.last_anchor_count = len(data.distances)
            if data.position is not None:
                self.last_pose_time = now
                self.last_qf = data.position.qf

    # ------------------------------------------------------------------ #
    #  Sağlık Durumu (/diagnostics)
    # ------------------------------------------------------------------ #

    def _publish_status(self, _event=None):
        now = monotonic()
        port_open = self.ser is not None and self.ser.is_open
        with self.lock:
            meas_age = None if self.last_meas_time is None else now - self.last_meas_time
            pose_age = None if self.last_pose_time is None else now - self.last_pose_time
            anchors  = self.last_anchor_count
            qf       = self.last_qf
            rate     = 0.0 if not self.meas_interval else 1.0 / self.meas_interval
            count    = self.meas_count

        def fmt(v):
            return 'nan' if v is None else '{:.3f}'.format(v)

        # Mesafe ve konum ayrı değerlendirilir: anchor < 3 iken mesafeler gelmeye
        # devam eder ama konum hesaplanamaz; kontrolcü bunu görebilmeli.
        ranges_st = self._age_status('UWB mesafe', meas_age, port_open)
        ranges_st.values = [
            KeyValue(key='age_s',             value=fmt(meas_age)),
            KeyValue(key='rate_hz',           value='{:.1f}'.format(rate)),
            KeyValue(key='anchor_count',      value=str(anchors)),
            KeyValue(key='measurement_count', value=str(count)),
            KeyValue(key='port_open',         value=str(port_open)),
        ]
        pose_st = self._age_status('UWB konum', pose_age, port_open)
        pose_st.values = [
            KeyValue(key='age_s',          value=fmt(pose_age)),
            KeyValue(key='quality_factor', value=str(qf)),
            KeyValue(key='anchor_count',   value=str(anchors)),
        ]

        arr = DiagnosticArray()
        arr.header.stamp = rospy.Time.now()
        arr.status = [ranges_st, pose_st]
        self.diag_pub.publish(arr)

    def _age_status(self, what, age, port_open):
        st = DiagnosticStatus()
        st.name = '{}: {}'.format(rospy.get_name().lstrip('/'), what)
        st.hardware_id = self.serial_port
        if not port_open:
            st.level, st.message = DiagnosticStatus.ERROR, 'Seri port kapalı'
        elif age is None:
            st.level, st.message = DiagnosticStatus.ERROR, 'Henüz veri alınmadı'
        elif age > self.error_age:
            st.level, st.message = DiagnosticStatus.ERROR, 'Veri yok ({:.2f} sn)'.format(age)
        elif age > self.warn_age:
            st.level, st.message = DiagnosticStatus.WARN, 'Veri gecikiyor ({:.2f} sn)'.format(age)
        else:
            st.level, st.message = DiagnosticStatus.OK, 'OK'
        return st

    # ------------------------------------------------------------------ #
    #  Temizlik
    # ------------------------------------------------------------------ #

    def shutdown(self):
        self.running = False
        self.status_timer.shutdown()
        self.read_thread.join(timeout=2.0)
        if self.ser is not None and self.ser.is_open:
            try:
                # Önce Enter ile 'lec' akışını durdur, sonra cihazı API moduna
                # döndür; bir sonraki açılışta temiz başlasın
                self.ser.write(b'\r')
                time.sleep(0.2)
                self.ser.write(b'reset\r')
            except (serial.SerialException, OSError):
                pass
        self._close_serial()
        rospy.loginfo('Kapatıldı.')


# ====================================================================== #
def main():
    rospy.init_node('mdek1001_uwb_node')
    MDEK1001Node()
    rospy.spin()


if __name__ == '__main__':
    main()
