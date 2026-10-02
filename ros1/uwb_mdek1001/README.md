# uwb_mdek1001 (ROS 1)

MDEK1001 tag'inden seri port üzerinden mesafe ve konum okuyup ROS 1'e yayınlar.
Python 2.7 (Melodic) ve Python 3 (Noetic) ile çalışır.

## Kurulum (Jetson Nano)

```bash
# 0) Sistem bilgisi
lsb_release -a; rosversion -d; echo $ROS_PYTHON_VERSION

# 1) Paketi catkin çalışma alanına koyun: ~/catkin_ws/src/uwb_mdek1001
cd ~/catkin_ws/src/uwb_mdek1001
sed -i 's/\r$//' scripts/*.py launch/*.launch     # Windows satır sonlarını temizle
chmod +x scripts/uwb_node.py

# 2) Bağımlılık
sudo apt install python-serial      # Melodic (Python 2)
sudo apt install python3-serial     # Noetic (Python 3)

# 3) Seri port izni (sonra oturumu kapatıp açın)
sudo usermod -aG dialout $USER

# 4) Sabit cihaz adı /dev/uwb (+ ModemManager'ın portu ele geçirmesini engeller)
sudo cp udev/99-uwb-mdek1001.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/uwb

# 5) Derleme
cd ~/catkin_ws && catkin_make && source devel/setup.bash
```

Noetic'te `python` komutu yoksa: `sudo apt install python-is-python3`

## Çalıştırma

```bash
roslaunch uwb_mdek1001 uwb.launch port:=/dev/uwb
```

## Kontrol

```bash
rostopic hz /uwb/pose            # ~10 Hz
rostopic echo -n1 /uwb/pose
rostopic echo /diagnostics       # "UWB mesafe" ve "UWB konum": OK / WARN / ERROR
rostopic echo /uwb/raw           # cihazdan gelen ham satırlar (sorun ayıklama)
```

## Topic'ler

| Topic | Tip | Açıklama |
|---|---|---|
| `/uwb/pose` | `geometry_msgs/PoseWithCovarianceStamped` | Tag konumu (`map` frame). z ve yönelim belirsizliği çok büyük (2B kullanın) |
| `/uwb/range` | `sensor_msgs/Range` | Anchor başına mesafe; `frame_id = uwb_link/<anchor_id>` |
| `/uwb/all_ranges` | `std_msgs/Float32MultiArray` | Satırdaki tüm mesafeler, anchor ID sırasıyla |
| `/uwb/raw` | `std_msgs/String` | Ham satır |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Ölçüm/konum yaşı, hız, anchor sayısı, qf |

Her ölçüm geldiği anda, okunduğu anın zaman damgasıyla bir kez yayınlanır; eski veri tekrar edilmez.

## Sık karşılaşılan sorunlar

| Belirti | Çözüm |
|---|---|
| `Seri port açılamadı: Permission denied` | `dialout` grubu (adım 3) ve oturumu yeniden açma |
| `Seri port açılamadı: No such file` | `ls /dev/ttyACM* /dev/uwb`; doğru portu `port:=` ile verin |
| `Device or resource busy` | minicom / başka bir node portu açık tutuyor |
| `/usr/bin/env: 'python\r'` | Adım 1'deki `sed` komutu |
| `veri gelmedi, lec yeniden başlatılıyor` | Takılı cihaz tag mı? (minicom'da `si` → `mode: tn`) |
| Mesafe var, konum yok (`UWB konum` WARN/ERROR) | 3'ten az anchor görülüyor veya anchor koordinatları hatalı |
