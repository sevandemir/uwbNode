#!/usr/bin/env python3
"""
MDEK1001 seri port testi (ROS gerektirmez; Windows / Linux).

uwb_node ile aynı başlatma sırasını uygular ('\\r\\r' → 1 sn → 'lec\\r'),
gelen her satırı node'un kullandığı ayrıştırıcıyla çözer, ölçüm hızını ve
ölçümler arası boşlukları raporlar. Tüm ham satırlar zaman damgasıyla
log dosyasına yazılır.

Kullanım:
  py tools/serial_test.py --list
  py tools/serial_test.py --port COM5
  py tools/serial_test.py --port COM5 --duration 60 --no-init
"""

import argparse
import datetime
import os
import statistics
import sys
import time

import serial
import serial.tools.list_ports

# Node'un ayrıştırıcısını doğrudan kullan
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'src', 'uwb_mdek1001'))
from uwb_mdek1001.lec_parser import parse_lec_line  # noqa: E402

WARN_GAP = 0.3   # sn, node'daki warn_age varsayılanı
ERROR_GAP = 1.0  # sn, node'daki error_age varsayılanı


class Logger:
    def __init__(self, path):
        self.f = open(path, 'w', encoding='utf-8')
        self.t0 = time.perf_counter()

    def __call__(self, text, echo=True):
        line = f'[{time.perf_counter() - self.t0:9.3f}] {text}'
        self.f.write(line + '\n')
        self.f.flush()
        if echo:
            print(line)

    def close(self):
        self.f.close()


def list_ports():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print('Hiç seri port bulunamadı. Cihaz takılı mı, J-Link sürücüsü kurulu mu?')
    for p in ports:
        print(f'{p.device:10s} {p.description}  [{p.hwid}]')


def read_for(ser, seconds):
    """Belirli süre gelen her şeyi ham olarak toplar (banner / prompt görmek için)."""
    end = time.monotonic() + seconds
    buf = b''
    while time.monotonic() < end:
        buf += ser.read(ser.in_waiting or 1)
    return buf


def start_lec(ser, log):
    log('>>> Shell moduna geçiş: \\r\\r gönderiliyor')
    ser.reset_input_buffer()
    ser.write(b'\r\r')
    reply = read_for(ser, 1.0)
    log(f'<<< 1 sn içinde gelen cevap ({len(reply)} bayt): {reply!r}')
    if b'dwm>' in reply:
        log('    OK: "dwm>" prompt görüldü, cihaz shell modunda.')
    elif not reply:
        log('    UYARI: hiç cevap yok. Port doğru mu? Cihaz zaten shell modunda ve sessiz olabilir.')
    else:
        log('    UYARI: "dwm>" prompt görülmedi.')

    # Sistem bilgisi: firmware sürümü ve modu (tag 'tn' / anchor 'an') burada görünür
    log(">>> 'si\\r' gönderiliyor (sistem bilgisi)")
    ser.write(b'si\r')
    reply = read_for(ser, 1.0)
    log(f'<<< si cevabı ({len(reply)} bayt):')
    for ln in reply.decode('utf-8', errors='replace').splitlines():
        if ln.strip():
            log(f'    | {ln.rstrip()}')

    log(">>> 'lec\\r' gönderiliyor")
    ser.write(b'lec\r')


def reconnect(open_port, log, deadline):
    """Port kopunca 1 sn aralıkla yeniden açmayı dener; açılırsa lec'i yeniden başlatır."""
    t0 = time.monotonic()
    while time.monotonic() < deadline:
        try:
            ser = open_port()
        except serial.SerialException:
            time.sleep(1.0)
            continue
        log(f'+++ Port yeniden açıldı ({time.monotonic() - t0:.1f} sn sonra)')
        start_lec(ser, log)
        return ser
    return None


def run(ser, duration, log, do_init=True, open_port=None):
    """Dinler ve özet basar. Bağlantı koparsa open_port ile yeniden bağlanır; son portu döndürür."""
    if do_init:
        start_lec(ser, log)

    stats = {'lines': 0, 'meas': 0, 'pose': 0, 'unparsed': 0}
    unparsed_samples = []
    meas_times = []
    anchor_counts = []
    qfs = []
    pose_times = []
    disconnects = []
    t_start = time.monotonic()
    next_report = t_start + 1.0

    log(f'--- {duration:.0f} sn dinleniyor (Ctrl+C ile erken bitirebilirsiniz) ---')
    try:
        while time.monotonic() - t_start < duration:
            try:
                raw = ser.readline()
            except serial.SerialException as e:
                disconnects.append(time.monotonic() - t_start)
                log(f'!!! BAĞLANTI KOPTU: {e}')
                log('    (USB kablosu oynadı, cihaz resetlendi veya Windows USB güç tasarrufu portu kapattı olabilir)')
                try:
                    ser.close()
                except serial.SerialException:
                    pass
                ser = reconnect(open_port, log, t_start + duration) if open_port else None
                if ser is None:
                    log('    Süre içinde yeniden bağlanılamadı.')
                    break
                continue
            now = time.monotonic()
            line = raw.decode('utf-8', errors='replace').strip()
            if line:
                stats['lines'] += 1
                data = parse_lec_line(line)
                if data is None or (not data.distances and data.position is None):
                    stats['unparsed'] += 1
                    if len(unparsed_samples) < 20:
                        unparsed_samples.append(line)
                    log(f'RAW  {line!r}   → ayrıştırılmadı')
                else:
                    stats['meas'] += 1
                    meas_times.append(now)
                    anchor_counts.append(len(data.distances))
                    pos = ''
                    if data.position is not None:
                        stats['pose'] += 1
                        qfs.append(data.position.qf)
                        pose_times.append(now)
                        p = data.position
                        pos = f'  POS=({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) qf={p.qf}'
                    log(f'RAW  {line}', echo=False)
                    log(f'MEAS {data.distances}{pos}')

            if now >= next_report:
                recent = [t for t in meas_times if now - t <= 1.0]
                age = now - meas_times[-1] if meas_times else float('nan')
                log(f'... son 1 sn: {len(recent)} ölçüm, son ölçüm yaşı {age:.2f} sn')
                next_report = now + 1.0
    except KeyboardInterrupt:
        log('Kullanıcı durdurdu.')

    # --- Özet ---
    elapsed = time.monotonic() - t_start
    log('=' * 60)
    log(f'Süre                 : {elapsed:.1f} sn')
    log(f'Toplam satır         : {stats["lines"]}')
    log(f'Ölçüm satırı         : {stats["meas"]}  (konum içeren: {stats["pose"]})')
    log(f'Ayrıştırılamayan     : {stats["unparsed"]}')
    if len(meas_times) >= 2:
        gaps = [b - a for a, b in zip(meas_times, meas_times[1:])]
        first_delay = meas_times[0] - t_start
        log(f'İlk ölçüme kadar     : {first_delay:.2f} sn')
        log(f'Ortalama hız         : {(len(meas_times) - 1) / (meas_times[-1] - meas_times[0]):.2f} Hz')
        log(f'Aralık ort / medyan  : {statistics.mean(gaps) * 1000:.0f} / {statistics.median(gaps) * 1000:.0f} ms')
        log(f'Aralık min / max     : {min(gaps) * 1000:.0f} / {max(gaps) * 1000:.0f} ms')
        log(f'Aralık std sapma     : {statistics.pstdev(gaps) * 1000:.0f} ms')
        log(f'> {WARN_GAP * 1000:.0f} ms boşluk      : {sum(g > WARN_GAP for g in gaps)}')
        log(f'> {ERROR_GAP * 1000:.0f} ms boşluk     : {sum(g > ERROR_GAP for g in gaps)}')
        log(f'Anchor sayısı min/max: {min(anchor_counts)} / {max(anchor_counts)}')
        if qfs:
            log(f'Kalite (qf) min/ort/max: {min(qfs)} / {statistics.mean(qfs):.0f} / {max(qfs)}')
        if pose_times:
            # Konumsuz geçen süreler: başlangıç → ilk konum, konumlar arası, son konum → bitiş
            pts = [t_start] + pose_times + [time.monotonic()]
            pgaps = [b - a for a, b in zip(pts, pts[1:])]
            log(f'Konum oranı          : %{100 * stats["pose"] / stats["meas"]:.0f}')
            log(f'Konumsuz en uzun süre: {max(pgaps) * 1000:.0f} ms')
            log(f'Konumsuz > {WARN_GAP * 1000:.0f} ms   : {sum(g > WARN_GAP for g in pgaps)}')
            log(f'Konumsuz > {ERROR_GAP * 1000:.0f} ms  : {sum(g > ERROR_GAP for g in pgaps)}')
        else:
            log('Konum                : hiç gelmedi (en az 3 anchor ve tutarlı anchor koordinatları gerekir)')
    else:
        log('SONUÇ: ölçüm alınamadı (veya tek ölçüm). Yukarıdaki RAW satırlarına bakın.')
    if disconnects:
        log(f'Bağlantı kopması     : {len(disconnects)} kez, t = ' + ', '.join(f'{t:.1f}' for t in disconnects) + ' sn')
    if unparsed_samples:
        log('Ayrıştırılamayan satır örnekleri:')
        for s in unparsed_samples:
            log(f'    {s!r}')
    return ser


def main():
    ap = argparse.ArgumentParser(description='MDEK1001 seri port testi')
    ap.add_argument('--list', action='store_true', help='seri portları listele ve çık')
    ap.add_argument('--port', help='ör. COM5 veya /dev/ttyACM0')
    ap.add_argument('--baud', type=int, default=115200)
    ap.add_argument('--duration', type=float, default=30.0, help='dinleme süresi (sn)')
    ap.add_argument('--no-init', action='store_true',
                    help="başlatma komutlarını gönderme, sadece dinle (cihaz zaten 'lec' akışındaysa)")
    ap.add_argument('--no-reset', action='store_true', help="çıkışta 'reset' gönderme")
    ap.add_argument('--log', help='log dosyası (varsayılan: uwb_test_<tarih>.log)')
    args = ap.parse_args()

    if args.list:
        list_ports()
        return
    if not args.port:
        ap.error('--port gerekli. Portları görmek için: --list')

    log_path = args.log or datetime.datetime.now().strftime('uwb_test_%Y%m%d_%H%M%S.log')
    log = Logger(log_path)
    log(f'Port={args.port} baud={args.baud} süre={args.duration}s init={not args.no_init} '
        f'python={sys.version.split()[0]} pyserial={serial.__version__}')

    def open_port():
        return serial.serial_for_url(args.port, baudrate=args.baud, timeout=1.0)

    try:
        ser = open_port()
    except serial.SerialException as e:
        log(f'HATA: port açılamadı: {e}')
        log('  - Port adı doğru mu? (--list)  - Tera Term/PuTTY/minicom gibi başka bir program portu açık tutuyor olabilir.')
        log.close()
        sys.exit(1)

    try:
        ser = run(ser, args.duration, log, do_init=not args.no_init, open_port=open_port) or ser
    finally:
        if not args.no_reset and ser.is_open:
            try:
                ser.write(b'\r')
                time.sleep(0.2)
                ser.write(b'reset\r')
                log("Çıkış: 'reset' gönderildi (cihaz API moduna döner).")
            except serial.SerialException:
                pass
        ser.close()
        log(f'Log dosyası: {os.path.abspath(log_path)}')
        log.close()


if __name__ == '__main__':
    main()
