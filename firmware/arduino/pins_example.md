### Contoh Wiring Dual Motor Driver

Anda bisa menggunakan TB6612FNG atau L298N. Berikut contoh untuk TB6612:

- STBY → 5V (atau gunakan pin `PIN_STBY` dan set HIGH)
- PWMA (PWM kiri) → D5
- AIN1 → D7
- AIN2 → D8
- PWMB (PWM kanan) → D6
- BIN1 → D9
- BIN2 → D10
- VMotor → 7–12V (sesuai motor), GND -> Common Ground
- VIN 5V → Arduino 5V (gunakan regulator terpisah jika arus besar)

Jika memakai L298N (tanpa STBY):
- ENA (PWM kiri) → D5
- IN1 → D7
- IN2 → D8
- ENB (PWM kanan) → D6
- IN3 → D9
- IN4 → D10
- 12V → Motor supply, 5V ke Arduino (opsional jika modul menyediakan 5V stabil)

Ubah define pin di `rover_firmware.ino` sesuai wiring Anda.


