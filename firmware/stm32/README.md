# STM32 Rover Firmware (Skeleton)

Template opsional jika Anda menggunakan STM32 (CubeIDE/HAL). Prinsip protokol sama dengan Arduino dan kini disertai kode siap pakai:

Perintah ASCII (mode stm32 dari aplikasi laptop):
- `MOVE FWD D=<cm> V=<0-255>`
- `MOVE BACK D=<cm> V=<0-255>`
- `TURN LEFT A=<deg> V=<0-255>`
- `TURN RIGHT A=<deg> V=<0-255>`
- `STOP`
- `HEARTBEAT`
- `CAL SET ms_per_cm=<int>`
- `CAL SET ms_per_deg=<int>`

Langkah implementasi:
1. Buat project CubeIDE untuk board Anda (mis. NUCLEO-F103RB).
2. Aktifkan UART (115200-8N1), timer PWM untuk 2 channel motor kiri/kanan.
3. Copy file berikut ke project:
   - `firmware/stm32/proto.h`, `firmware/stm32/proto.c`
   - `firmware/stm32/stm32_adapter.h`, `firmware/stm32/stm32_adapter.c`
4. Di `main.c`, setelah inisialisasi UART, panggil:
   ```c
   STM32A_Init(&huart1); // ganti huart sesuai project Anda
   ```
5. Di callback:
   ```c
   void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
     STM32A_RxCpltCallback(huart);
   }
   ```
6. Implementasikan fungsi motor Anda (atau biarkan override):
   ```c
   void Motor_Stop(void){ /* matikan PWM kiri/kanan */ }
   void Move_OpenLoop_cm(int dist_cm, int sp){ /* gerak lurus open-loop */ }
   void Turn_OpenLoop_deg(int deg, int sp, int left){ /* putar di tempat */ }
   ```
7. Selesai. STM32 akan mengeksekusi baris ASCII dari laptop.

Contoh struktur file:
```
Src/
  main.c
  motor.c                // motor_forward/back/turn_left/turn_right/stop
  proto.c                // parse_line()
  stm32_adapter.c        // RX IT + eksekusi
Inc/
  motor.h
  proto.h
  stm32_adapter.h
```

Catatan:
- Pastikan ground bersama antara driver motor dan board.
- Jika memakai level shifter untuk PWM/IN pin (tergantung tegangan).
- Encoder dukungan dapat ditambah nanti untuk closed-loop.


