## Integrasi Laptop → STM32 (Tanpa Ubah Firmware)

Dokumen ini menjelaskan cara menjalankan `autonomous_rover` sehingga perintah yang dikirim sudah sesuai format STM32 Anda (mode `stm32`).

### 1) Konfigurasi
Edit `config/rover_config.yaml`:
```
robot.port: "/dev/rfcomm0"     # atau /dev/ttyACM0 jika USB-CDC
robot.baud: 115200
protocol.mode: "stm32"         # "stm32" atau "standard"
```

### 2) Konektivitas HC‑05 ↔ STM32
- HC‑05 TX → STM32 RX
- HC‑05 RX → STM32 TX
- GND bersama
- Vcc sesuai modul (biasanya 5V)
- Baud 115200 (default proyek)

Pairing di laptop:
```
./scripts/setup_bluetooth.sh <BT_MAC> 1
```
Device akan tampil sebagai `/dev/rfcomm0`.

### 3) Format Perintah (Mode stm32)
- Maju: `F <cm> <speed>`
- Mundur: `B <cm> <speed>`
- Putar kiri: `TL <deg> <speed>`
- Putar kanan: `TR <deg> <speed>`
- Stop: `S`
- Heartbeat: `HB` (opsional)

Contoh output dari program:
```
F 15 170
TL 12 150
S
```
Catatan: setiap baris berakhir newline (`\\n`).

### 4) Jalankan Program
```
./scripts/run_rover.sh
```
Atau override kamera:
```
./scripts/run_rover.sh http://IP_HP:8080/video
```

### 5) Uji Tanpa Robot (Pseudo‑Serial)
Instal alat:
```
sudo apt install -y socat screen
```
Buat dua pseudo‑TTY:
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
```
Catat dua device (mis. `/dev/pts/3` dan `/dev/pts/4`).
Set `robot.port: "/dev/pts/3"` lalu jalankan program.
Buka terminal lain untuk melihat output:
```
screen /dev/pts/4 115200
# atau
cat -v < /dev/pts/4
```
Anda akan melihat baris `F/B/TL/TR/S`.

### 6) Checklist Tim Elektrikal
1. Pasang HC‑05 ke UART STM32 (115200), ground bersama.
2. Firmware STM32 membaca baris perintah di atas (tanpa perlu diubah).
3. Laptop pair HC‑05 → `/dev/rfcomm0`.
4. Jalankan `autonomous_rover` dan verifikasi perintah diterima.


