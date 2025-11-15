## ROVER QUICK START

1) Flash firmware ke Arduino
- Buka `firmware/arduino/rover_firmware.ino` di Arduino IDE.
- Sesuaikan pin di atas file (TB6612/L298N).
- Upload ke board (pilih port dan board yang benar).

2) Konfigurasi koneksi robot dan kamera
- Edit `config/rover_config.yaml`:
  - `camera.url`: URL IP camera (HP)
  - `robot.port`: `/dev/ttyUSB0` (USB) atau `/dev/rfcomm0` (Bluetooth)
  - `robot.baud`: 115200
- Jika Bluetooth (HC-05/06):
  ```bash
  ./scripts/setup_bluetooth.sh <BT_MAC> 1
  # contoh /dev/rfcomm0 akan dibuat
  ```

3) Jalankan sistem
```bash
./scripts/run_rover.sh
# atau override URL kamera:
./scripts/run_rover.sh http://192.168.1.100:8080/video
```

Tips:
- Kalibrasi jarak/derajat di `config/rover_config.yaml` setelah tes awal.
- File `visited_markers.json` akan menyimpan marker yang sudah dikunjungi.


