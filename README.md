# 🤖 Autonomous Rover - Computer Vision System

[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-blue.svg)](https://opencv.org/)
[![C++](https://img.shields.io/badge/C++-17-green.svg)](https://isocpp.org/)
[![CMake](https://img.shields.io/badge/CMake-3.0+-red.svg)](https://cmake.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)]()

Sistem navigasi autonomous rover dengan line following, ArUco marker detection, dan obstacle avoidance menggunakan OpenCV dan STM32/Arduino integration.

---

## 🚀 Quick Start

### Jalankan Autonomous Rover

```bash
# Compile project
cd build
cmake ..
make autonomous_rover -j$(nproc)

# Run dengan webcam
./autonomous_rover

# Atau gunakan script
cd ..
./scripts/run_rover.sh

# Run dengan custom camera URL
./scripts/run_rover.sh http://192.168.1.100:8080/video
```

---

## ✨ Fitur Utama

### 🤖 Autonomous Rover System

- 🔍 **Line Following** - White & yellow line detection dengan HSV filtering
- 🎯 **ArUco Marker Navigation** - Detection, tracking, dan decision making
- 🚧 **Obstacle Detection** - Black object detection untuk menghindari halangan
- 🎨 **Boundary Detection** - Edge detection untuk mencegah keluar jalur
- 🔊 **Voice Guidance (eSpeak)** - Text-to-speech untuk notifikasi realtime
- 📊 **Decision Logging** - CSV logging untuk analisis performa
- 🤝 **Robot Integration** - Serial communication dengan STM32/Arduino via HC-05 Bluetooth
- � **Smart Control** - Toggle-based command system untuk kontrol robot

### 🧩 Modular Architecture

- **Path Planner** - A\* pathfinding algorithm
- **Obstacle Detector** - Real-time obstacle detection
- **Robot Bridge** - Hardware abstraction layer
- **Serial Transport** - Reliable serial communication
- **YOLO Detector** - Optional YOLOv5 object detection (jika ONNX Runtime tersedia)

### 🎛️ Configuration System

- YAML-based configuration (`config/rover_config.yaml`)
- Runtime parameter tuning
- Multiple camera source support (webcam, IP camera, video file)

---

## 📖 Dokumentasi

### 🎯 Rover Integration

- **[ROVER_INTEGRATION.md](docs/ROVER_INTEGRATION.md)** - Panduan integrasi robot
- **[ROVER_QUICK_START.md](docs/ROVER_QUICK_START.md)** - Quick start guide
- **[STM32_INTEGRATION.md](docs/STM32_INTEGRATION.md)** - STM32 integration details
- **[STM32_ELECTRICAL_GUIDE.md](docs/STM32_ELECTRICAL_GUIDE.md)** - Wiring dan setup hardware
- **[PROGRAMMER_NEXT_STEPS.md](docs/PROGRAMMER_NEXT_STEPS.md)** - Development guide

---

## 🛠️ Installation & Setup

### Prerequisites

```bash
sudo apt-get update
sudo apt-get install -y \
    libopencv-dev \
    libopencv-contrib-dev \
    cmake \
    g++ \
    build-essential \
    espeak \
    libespeak-dev
```

> **Note:** eSpeak + libespeak-dev diperlukan untuk fitur Text-to-Speech (TTS).

### Optional: ONNX Runtime (untuk YOLOv5 Detection)

```bash
# Download ONNX Runtime
wget https://github.com/microsoft/onnxruntime/releases/download/v1.16.0/onnxruntime-linux-x64-1.16.0.tgz
tar -xzf onnxruntime-linux-x64-1.16.0.tgz
sudo cp -r onnxruntime-linux-x64-1.16.0/include/* /usr/local/include/
sudo cp -r onnxruntime-linux-x64-1.16.0/lib/* /usr/local/lib/
sudo ldconfig
```

### Compile Project

```bash
# Clone repository
git clone https://github.com/irsyad-a/ArUco-Detec-Autonomous.git
cd ArUco-Detec-Autonomous

# Create build directory
mkdir -p build
cd build

# Configure and compile
cmake ..
make autonomous_rover -j$(nproc)
```

### Verify Installation

```bash
cd build
ls -lh autonomous_rover
./autonomous_rover --help
```

### Voice Guidance

Saat menjalankan `autonomous_rover`, Anda akan mendapat prompt:

```
Aktifkan suara deteksi? [Y/n]:
```

- Tekan **Enter** atau `y` untuk mengaktifkan TTS
- Masukkan `n` untuk mode silent

### HC-05 Bluetooth Control

Sistem menggunakan metode **toggle control**:

- Karakter `w`, `a`, `s`, `d`, `q`, `e`, `g`, `p` akan toggle ON/OFF
- Sistem otomatis mematikan perintah lama sebelum aktivasi perintah baru
- Instruksi STOP mengirim ulang karakter yang sama

---

## 📱 IP Camera Setup

### Quick Setup (Android):

1. **Download** "IP Webcam" dari Play Store
2. **Buka app** → Scroll down → **"Start server"**
3. **Catat URL** (contoh: `http://192.168.1.100:8080/video`)
4. **Run dengan URL:**
   ```bash
   ./scripts/run_rover.sh http://192.168.1.100:8080/video
   ```

### Configuration File

Edit `config/rover_config.yaml`:

```yaml
camera:
  source: "http://192.168.1.100:8080/video" # IP camera URL
  # source: 0  # or use webcam
  width: 640
  height: 480
  fps: 30
```

---

## 🎮 Usage Examples

### Example 1: Run dengan Webcam

```bash
./scripts/run_rover.sh
# Atau
cd build
./autonomous_rover
```

### Example 2: Run dengan IP Camera

```bash
./scripts/run_rover.sh http://192.168.1.100:8080/video
```

### Example 3: Run dengan Video File

```bash
cd build
./autonomous_rover /path/to/video.mp4
```

### Keyboard Controls

Saat program berjalan:

- `q` - Quit/keluar dari program
- `r` - Reset decision state
- `m` - Toggle map window
- `d` - Toggle debug mode
- `v` - Toggle verbose logging
- `s` - Save screenshot
- `p` - Pause/resume

### Robot Commands (via Serial/Bluetooth)

Jika terhubung dengan robot via HC-05:

- `w` - Maju (Forward)
- `s` - Mundur (Backward)
- `a` - Belok kiri (Turn Left)
- `d` - Belok kanan (Turn Right)
- `q` - Rotate kiri (Rotate Left)
- `e` - Rotate kanan (Rotate Right)
- `g` - Stop semua motor
- `p` - Pause toggle

---

## 🔧 Scripts & Tools

### 1. Run Rover (`scripts/run_rover.sh`)

Main script untuk compile dan run autonomous rover.

```bash
./scripts/run_rover.sh [camera_url]
```

### 2. Send Character via Bluetooth (`scripts/send_char.sh`)

Kirim single character command ke robot via serial/bluetooth.

```bash
./scripts/send_char.sh w  # Forward
./scripts/send_char.sh s  # Backward
```

### 3. Setup Bluetooth (`scripts/setup_bluetooth.sh`)

Setup dan konfigurasi HC-05 Bluetooth module.

```bash
./scripts/setup_bluetooth.sh
```

---

## 📊 Project Structure

```
ArUco-Detec-Autonomous/
│
├── 📁 src/                          # Source code
│   ├── autonomous_rover.cpp         # ⭐ Main autonomous system
│   ├── robot_bridge.{cpp,hpp}       # Robot hardware abstraction
│   ├── serial_transport.{cpp,hpp}   # Serial communication
│   ├── path_planner.{cpp,hpp}       # A* pathfinding
│   ├── obstacle_detector.{cpp,hpp}  # Obstacle detection
│   ├── yolo_detector.{cpp,hpp}      # YOLO object detection
│   └── command_protocol.hpp         # Command definitions
│
├── 📁 build/                        # Compiled binaries
│   ├── autonomous_rover             # ⭐ Main executable
│   ├── autonomous_navigation_log.txt
│   ├── decisions_log.csv
│   └── visited_markers.json
│
├── 📁 config/                       # Configuration files
│   ├── rover_config.yaml            # Main config
│   └── rover_config_examples.yaml   # Example configs
│
├── � docs/                         # Documentation
│   ├── ROVER_INTEGRATION.md         # Integration guide
│   ├── ROVER_QUICK_START.md         # Quick start
│   ├── STM32_INTEGRATION.md         # STM32 guide
│   ├── STM32_ELECTRICAL_GUIDE.md    # Wiring guide
│   └── PROGRAMMER_NEXT_STEPS.md     # Development guide
│
├── � scripts/                      # Utility scripts
│   ├── run_rover.sh                 # ⭐ Main launcher
│   ├── send_char.sh                 # Send BT command
│   └── setup_bluetooth.sh           # BT setup
│
├── � firmware/                     # Robot firmware
│   ├── stm32/                       # STM32 code
│   └── arduino/                     # Arduino code
│
├── 📖 README.md                     # This file
└── 📄 CMakeLists.txt                # Build configuration
```

---

## 🎯 Development Workflow

### For First-Time Users:

1. **Clone & Build**

   ```bash
   git clone https://github.com/irsyad-a/ArUco-Detec-Autonomous.git
   cd ArUco-Detec-Autonomous
   mkdir build && cd build
   cmake ..
   make autonomous_rover -j$(nproc)
   ```

2. **Test dengan Webcam**

   ```bash
   ./autonomous_rover
   ```

3. **Configure untuk Robot**

   ```bash
   # Edit config/rover_config.yaml
   # Setup Bluetooth (jika menggunakan HC-05)
   ../scripts/setup_bluetooth.sh
   ```

4. **Read Documentation**
   - `docs/ROVER_QUICK_START.md` - Quick start guide
   - `docs/ROVER_INTEGRATION.md` - Integration details
   - `docs/STM32_INTEGRATION.md` - Hardware integration

### For Developers:

1. **Understand Architecture:**

   - `src/autonomous_rover.cpp` - Main state machine & logic
   - `src/robot_bridge.cpp` - Hardware abstraction layer
   - `src/path_planner.cpp` - A\* pathfinding algorithm
   - `src/obstacle_detector.cpp` - Obstacle detection module

2. **Modify Parameters:**

   ```yaml
   # Edit config/rover_config.yaml
   line_following:
     white_line:
       h_min: 0
       s_min: 0
       v_min: 200
   ```

3. **Build & Test:**

   ```bash
   cd build
   make autonomous_rover -j$(nproc)
   ./autonomous_rover
   ```

4. **Debug:**

   ```bash
   # Enable verbose logging
   ./autonomous_rover --verbose

   # Check logs
   tail -f autonomous_navigation_log.txt
   ```

---

## 🐛 Troubleshooting

### Issue: Camera tidak terbuka

**Solution:**

```bash
# 1. Check camera device
ls -l /dev/video*

# 2. Test dengan v4l2
v4l2-ctl --list-devices

# 3. Try different camera index
./autonomous_rover 0  # atau 1, 2, dst
```

### Issue: IP Camera tidak connect

**Solution:**

```bash
# 1. Check network connectivity
ping <IP_CAMERA>

# 2. Test di browser
firefox http://<IP>:8080/video

# 3. Try alternative URL
./autonomous_rover http://<IP>:8080/videofeed
```

### Issue: Line tidak terdeteksi

**Solution:**

- Sesuaikan HSV threshold di `config/rover_config.yaml`
- Pastikan pencahayaan cukup terang dan merata
- Test dengan debug mode: `./autonomous_rover --debug`
- Check HSV values: Window "HSV Adjustments"

### Issue: Compile error

**Solution:**

```bash
# Install all dependencies
sudo apt-get install -y libopencv-dev libopencv-contrib-dev \
    cmake g++ build-essential espeak libespeak-dev

# Clean rebuild
cd build
rm -rf *
cmake ..
make autonomous_rover -j$(nproc)
```

### Issue: Bluetooth tidak connect

**Solution:**

```bash
# 1. Check bluetooth device
ls -l /dev/rfcomm*

# 2. Pair HC-05
./scripts/setup_bluetooth.sh

# 3. Test connection
./scripts/send_char.sh w
```

### Issue: Robot tidak merespon command

**Solution:**

- Check serial port di `config/rover_config.yaml`
- Verify baud rate (default: 9600)
- Test dengan `./scripts/send_char.sh`
- Check firmware di robot (STM32/Arduino)

---

## 💡 Tips & Best Practices

### Hardware Setup:

- ✅ Gunakan power supply yang stabil untuk robot
- ✅ Pastikan HC-05 terpasang dengan benar (TX->RX, RX->TX)
- ✅ Ground semua komponen ke common ground
- ✅ Test koneksi serial sebelum integration

### Line Following:

- 🎯 Gunakan tape berwarna putih/kuning dengan kontras tinggi
- 📏 Lebar garis minimal 2-3 cm
- 💡 Pencahayaan merata, hindari shadow
- � Background gelap untuk white line, terang untuk yellow line

### ArUco Markers:

- 📐 Print marker minimal 5x5 cm
- 🖨️ Quality printing (hitam pekat, putih bersih)
- 📏 Jarak deteksi optimal: 20-100 cm
- � Pastikan marker flat (tidak terlipat)

### Development:

- 🔧 Use configuration file untuk parameter tuning
- 📝 Enable logging untuk debugging
- 💾 Commit changes incrementally
- 🧪 Test di simulator sebelum real robot
- 📊 Monitor decision logs untuk analisis performa

---

## 🎓 Learning Resources

### Internal Documentation:

- [docs/ROVER_INTEGRATION.md](docs/ROVER_INTEGRATION.md) - Complete integration guide
- [docs/ROVER_QUICK_START.md](docs/ROVER_QUICK_START.md) - Quick start guide
- [docs/STM32_INTEGRATION.md](docs/STM32_INTEGRATION.md) - STM32 programming guide
- [docs/STM32_ELECTRICAL_GUIDE.md](docs/STM32_ELECTRICAL_GUIDE.md) - Wiring & hardware
- [docs/PROGRAMMER_NEXT_STEPS.md](docs/PROGRAMMER_NEXT_STEPS.md) - Development guide

### External Resources:

- [OpenCV Documentation](https://docs.opencv.org/4.x/)
- [OpenCV ArUco Tutorial](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)
- [ArUco Original Paper](https://www.uco.es/investiga/grupos/ava/node/26)
- [STM32 HAL Documentation](https://www.st.com/en/embedded-software/stm32cube-mcu-mpu-packages.html)
- [Serial Communication Guide](https://www.cmrr.umn.edu/~strupp/serial.html)

### Algorithms:

- **A\* Pathfinding**: Path planning untuk navigasi
- **HSV Color Filtering**: Line detection
- **Contour Detection**: Obstacle & boundary detection
- **ArUco Detection**: Marker-based localization

---

## 🚀 Features & Roadmap

### ✅ Implemented Features:

- [x] **Line Following** - White & yellow line detection
- [x] **ArUco Navigation** - Marker detection & tracking
- [x] **Obstacle Detection** - Black object detection
- [x] **Boundary Detection** - Edge detection
- [x] **Voice Guidance** - eSpeak TTS integration
- [x] **Robot Integration** - Serial/Bluetooth communication
- [x] **Path Planning** - A\* algorithm implementation
- [x] **Decision Logging** - CSV logging system
- [x] **Configuration System** - YAML-based config
- [x] **Multiple Camera Support** - Webcam, IP cam, video file
- [x] **YOLO Detection** (Optional) - Object detection with YOLOv5

### 🔄 Planned Improvements:

- [ ] SLAM (Simultaneous Localization and Mapping)
- [ ] Advanced obstacle avoidance with sensor fusion
- [ ] Multi-robot coordination
- [ ] Web dashboard untuk monitoring
- [ ] Mobile app remote control
- [ ] Machine learning untuk decision making
- [ ] GPS integration untuk outdoor navigation
- [ ] ROS2 integration

---

## 👨‍💻 Technical Details

**Repository:** [irsyad-a/ArUco-Detec-Autonomous](https://github.com/irsyad-a/ArUco-Detec-Autonomous)  
**Version:** 3.0.0  
**Last Updated:** November 15, 2025

**Core Technologies:**

- **OpenCV 4.x** - Computer vision library
- **C++17** - Main programming language
- **CMake 3.0+** - Build system
- **eSpeak** - Text-to-speech engine
- **YAML-CPP** - Configuration parsing
- **ONNX Runtime** (Optional) - YOLOv5 inference

**Hardware Support:**

- **STM32** - Microcontroller platform
- **Arduino** - Alternative microcontroller
- **HC-05** - Bluetooth communication module
- **Camera** - Webcam, IP camera, or video file

**Platform:**

- **Linux** (Ubuntu/Debian)
- Tested on: Ubuntu 22.04 LTS

---

## 🙏 Acknowledgments

**Libraries & Tools:**

- OpenCV - Computer vision foundation
- ArUco - Fiducial marker system
- eSpeak - Open source TTS engine
- ONNX Runtime - ML inference engine

**Resources:**

- OpenCV documentation & tutorials
- ArUco research papers
- STM32 HAL libraries
- Community contributions

---

## 📄 License

MIT License - Free to use for educational and commercial purposes.

See [LICENSE](LICENSE) file for details.

---

## 📞 Support & Issues

**Need Help?**

1. ✅ Check [docs/ROVER_QUICK_START.md](docs/ROVER_QUICK_START.md)
2. ✅ Read [docs/ROVER_INTEGRATION.md](docs/ROVER_INTEGRATION.md)
3. ✅ Look at Troubleshooting section above
4. ✅ Check log files: `build/autonomous_navigation_log.txt`

**Found a Bug?**

- Open an issue on GitHub: [Issues](https://github.com/irsyad-a/ArUco-Detec-Autonomous/issues)
- Include log files and error messages
- Describe steps to reproduce

**Want to Contribute?**

- Fork the repository
- Create feature branch
- Submit pull request
- Follow coding standards

---

## 🎊 Quick Start Summary

### Get Started in 3 Steps:

```bash
# 1. Clone & Build
git clone https://github.com/irsyad-a/ArUco-Detec-Autonomous.git
cd ArUco-Detec-Autonomous
./scripts/run_rover.sh

# 2. Configure (optional)
nano config/rover_config.yaml

# 3. Run!
./scripts/run_rover.sh
```

### With IP Camera:

```bash
# Setup IP Webcam app on Android
./scripts/run_rover.sh http://192.168.1.100:8080/video
```

### Integration dengan Robot:

```bash
# Setup Bluetooth
./scripts/setup_bluetooth.sh

# Test command
./scripts/send_char.sh w

# Run autonomous
./scripts/run_rover.sh
```

---

## 🌟 Key Features Highlight

### 🤖 Autonomous Navigation

- **Line Following** dengan white/yellow line detection
- **ArUco Markers** untuk waypoint navigation
- **Obstacle Avoidance** dengan real-time detection
- **Smart Decision Making** dengan state machine

### 🔧 Flexible Integration

- **Multiple Camera Sources** - Webcam, IP camera, video file
- **Serial/Bluetooth Support** - HC-05 integration ready
- **YAML Configuration** - Easy parameter tuning
- **Modular Architecture** - Easy to extend

### 📊 Monitoring & Debug

- **Real-time Logging** - CSV decision logs
- **Voice Feedback** - eSpeak TTS notifications
- **Debug Windows** - HSV tuning, contour display
- **Performance Metrics** - FPS, decision timing

---

## 🎯 Command Reference

```bash
# Build & Run
./scripts/run_rover.sh                    # ⭐ Main launcher
./scripts/run_rover.sh <camera_url>       # With IP camera

# Build Only
cd build
cmake ..                                  # Configure
make autonomous_rover -j$(nproc)          # Compile

# Direct Run
cd build
./autonomous_rover                        # Default camera
./autonomous_rover 0                      # Webcam
./autonomous_rover /path/to/video.mp4     # Video file
./autonomous_rover <camera_url>           # IP camera

# Bluetooth/Serial
./scripts/setup_bluetooth.sh              # Setup HC-05
./scripts/send_char.sh <command>          # Send command (w/a/s/d)

# Configuration
nano config/rover_config.yaml             # Edit config
cp config/rover_config_examples.yaml config/rover_config.yaml
```

---

## 📚 Documentation Index

| Document                                                         | Description                        |
| ---------------------------------------------------------------- | ---------------------------------- |
| **[README.md](README.md)**                                       | **Main documentation (this file)** |
| [docs/ROVER_INTEGRATION.md](docs/ROVER_INTEGRATION.md)           | Complete integration guide         |
| [docs/ROVER_QUICK_START.md](docs/ROVER_QUICK_START.md)           | Quick start guide                  |
| [docs/STM32_INTEGRATION.md](docs/STM32_INTEGRATION.md)           | STM32 programming & setup          |
| [docs/STM32_ELECTRICAL_GUIDE.md](docs/STM32_ELECTRICAL_GUIDE.md) | Wiring & electrical setup          |
| [docs/PROGRAMMER_NEXT_STEPS.md](docs/PROGRAMMER_NEXT_STEPS.md)   | Development guide                  |

---

<div align="center">

## 🎉 Ready to Start!

**Autonomous Rover System**

_Line Following • ArUco Navigation • Obstacle Avoidance_

---

### Quick Start

```bash
git clone https://github.com/irsyad-a/ArUco-Detec-Autonomous.git
cd ArUco-Detec-Autonomous
./scripts/run_rover.sh
```

---

**Made with ❤️ using OpenCV & C++**

[![GitHub](https://img.shields.io/badge/GitHub-irsyad--a-blue?logo=github)](https://github.com/irsyad-a/ArUco-Detec-Autonomous)
[![Issues](https://img.shields.io/badge/Issues-Report-red?logo=github)](https://github.com/irsyad-a/ArUco-Detec-Autonomous/issues)
[![Pull Requests](https://img.shields.io/badge/PRs-Welcome-green?logo=github)](https://github.com/irsyad-a/ArUco-Detec-Autonomous/pulls)

_Last Updated: November 15, 2025 | Version 3.0.0_

</div>
