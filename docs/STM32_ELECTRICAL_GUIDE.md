## Panduan Elektrikal: Menghubungkan Laptop → STM32 (HC‑05) dan Menjalankan Autonomous

Tujuan: Membuat STM32 menerima perintah gerak dari laptop (autonomous_rover) tanpa mengubah logika motor yang sudah ada.

### 1) Koneksi Hardware
- Modul Bluetooth HC‑05 ↔ UART STM32:
  - HC‑05 TX → STM32 RX (USART yang dipilih)
  - HC‑05 RX → STM32 TX
  - GND HC‑05 → GND STM32 → GND driver motor (ground bersama)
  - Vcc HC‑05 → 5V (sesuai modul)
- Driver Motor L298N:
  - Sisi kiri ke ENA/IN1/IN2; sisi kanan ke ENB/IN3/IN4 (paralel untuk 4WD).
  - Sumber daya motor cukup (arus sesuai motor), ground satu titik.

### 2) Konfigurasi UART
- Baudrate 115200, 8N1, tanpa flow control.
- UART yang dipakai harus aktif di CubeMX (misal USART1).

### 3) Tambahkan File Firmware (Copy-Paste ke Project STM32)
Salin dari repo:
- `firmware/stm32/proto.h` dan `firmware/stm32/proto.c`
- `firmware/stm32/stm32_adapter.h` dan `firmware/stm32/stm32_adapter.c`

### 4) Integrasi Kode
- Di `main.c`, setelah `MX_USARTx_UART_Init()`:
  ```c
  STM32A_Init(&huart1); // ganti huart sesuai UART yang dipakai
  ```
- Di callback:
  ```c
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    STM32A_RxCpltCallback(huart);
  }
  ```
- Implement fungsi motor (atau gunakan yang sudah ada):
  ```c
  void Motor_Stop(void){ /* matikan PWM */ }
  void Move_OpenLoop_cm(int dist_cm, int sp){ /* gerak lurus open-loop */ }
  void Turn_OpenLoop_deg(int deg, int sp, int left){ /* putar di tempat */ }
  ```

### 5) Pairing HC‑05 di Laptop
Di laptop (Linux):
```
./scripts/setup_bluetooth.sh <BT_MAC> 1
```
Device serial akan muncul sebagai `/dev/rfcomm0`.

### 6) Konfigurasi Aplikasi Laptop
Edit `config/rover_config.yaml`:
```
robot.port: "/dev/rfcomm0"
robot.baud: 115200
protocol.mode: "stm32"
```

### 7) Jalankan
```
./scripts/run_rover.sh
```
Kamera HP aktif (IP camera). STM32 akan menerima perintah ASCII:
```
F <cm> <speed>
B <cm> <speed>
TL <deg> <speed>
TR <deg> <speed>
S
```

### 8) Uji Tanpa Gerak Motor
- Sementara log perintah masuk di UART monitor/semilog.
- Pastikan terlihat baris seperti: `F 15 170`, `TL 12 150`, `S`.
- Setelah yakin benar, hubungkan motor dan uji pelan-pelan.

Troubleshooting singkat:
- Tidak ada data di STM32: cek pairing, `robot.port`, baudrate, kabel RX/TX tertukar.
- Gerak terbalik: tukar kabel motor (OUT1/OUT2 atau OUT3/OUT4) atau sesuaikan fungsi motor.
- Terlalu kencang: turunkan `speed.forward/back/turn` di `rover_config.yaml`.


