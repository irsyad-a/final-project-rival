## ROVER INTEGRATION GUIDE

### Tujuan
Menghubungkan deteksi ArUco (kamera HP via IP) ke kontrol robot (Arduino/STM32) melalui Serial/Bluetooth sehingga rover bisa bergerak mengikuti instruksi navigasi.

### Arsitektur
- Perception: OpenCV + ArUco (`autonomous_rover`)
- Navigation: State machine memilih target marker, hindari marker yang sudah dikunjungi
- Bridge: Konversi instruksi (ID) → perintah robot (`RobotBridge`)
- Transport: Serial USB atau Bluetooth RFCOMM (`SerialTransport`)
- Firmware: Parser ASCII + kontrol motor open-loop

### Alur Data
1. Kamera (IP) → frame
2. Deteksi ArUco → daftar marker (ID, posisi, size)
3. Navigator → instruksi (MAJU/PUTAR/LOCKED/SCAN COMPLETE)
4. Bridge → baris ASCII (MOVE/TURN/STOP)
5. Serial → Firmware jalankan motor

### Komponen Utama
- `src/autonomous_rover.cpp` : main loop, deteksi, visited memory
- `src/robot_bridge.*`       : mapping instruksi → ASCII
- `src/serial_transport.*`   : pengiriman ke /dev/ttyUSB* /dev/rfcomm*
- `config/rover_config.yaml` : URL kamera, port serial, kalibrasi, speed
- `firmware/arduino/`        : firmware rover (Arduino)

### Protokol ASCII (baris diakhiri newline)
```
MOVE FWD D=20 V=150
MOVE BACK D=15 V=150
TURN LEFT A=30 V=130
TURN RIGHT A=20 V=130
STOP
HEARTBEAT
CAL SET ms_per_cm=80
CAL SET ms_per_deg=12
```
Respons (opsional): `OK`, `DONE`, `ERR <code>`

### Visited Memory
- File `visited_markers.json` menyimpan ID marker yang sudah dikunjungi.
- Navigator tidak akan memilih marker yang sudah visited.

### Koneksi
- USB Serial: `robot.port: "/dev/ttyUSB0"` (atau `/dev/ttyACM0`)
- Bluetooth: pair HC-05/06, lalu:
  ```bash
  ./scripts/setup_bluetooth.sh <BT_MAC> 1
  # hasil: /dev/rfcomm0
  ```
  Set `robot.port: "/dev/rfcomm0"`
  - Catatan baud HC-05: sering 9600 (data mode). Jika karakter acak/berantakan, coba set `robot.baud: 9600`.

### Troubleshooting
- Kamera gagal: uji `build/aruco_detection` terlebih dahulu.
- Serial permission: `sudo usermod -a -G dialout $USER` lalu re-login.
- Bluetooth bind gagal: pastikan pairing berhasil, ulangi `rfcomm release/bind`.
- Rover tidak bergerak: cek wiring motor dan level PWM, periksa power supply.

### Single-Char Control (opsional)
Aktifkan pengiriman karakter tunggal seperti tombol keyboard (tanpa newline) bersamaan dengan perintah ASCII.

- Aktifkan di `config/rover_config.yaml`:
  ```yaml
  robot.send_char_controls: true
  ```
- Pemetaan:
  - `MAJU` → `'w'`
  - `MUNDUR` → `'s'`
  - `PUTAR KIRI` → `'a'`
  - `PUTAR KANAN` → `'d'`
  - `STOP/LOCKED/SCAN COMPLETE` → `'x'`
- Uji manual tanpa menjalankan autonomous:
  ```bash
  ./scripts/send_char.sh /dev/rfcomm0 w 9600
  ```


