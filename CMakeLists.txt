cmake_minimum_required(VERSION 3.0)

project( cobaAja )

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)

find_package(OpenCV REQUIRED)

find_path(ESPEAK_INCLUDE_DIR
    NAMES espeak/speak_lib.h
    PATHS
        /usr/include
        /usr/local/include
)

find_library(ESPEAK_LIBRARY
    NAMES espeak
    PATHS
        /usr/lib
        /usr/local/lib
)

if(ESPEAK_LIBRARY)
    message(STATUS "✓ eSpeak TTS library ditemukan")
else()
    message(WARNING "✗ eSpeak library tidak ditemukan. Install paket 'espeak' dan 'libespeak-dev' untuk mengaktifkan TTS.")
endif()

# Find ONNX Runtime
# Try to find ONNX Runtime in common locations
set(ONNXRUNTIME_ROOT_PATH "" CACHE PATH "Root path of ONNX Runtime")
find_path(ONNXRUNTIME_INCLUDE_DIRS
    NAMES onnxruntime_cxx_api.h
    PATHS
        ${ONNXRUNTIME_ROOT_PATH}/include
        /usr/include/onnxruntime
        /usr/local/include/onnxruntime
        /opt/onnxruntime/include
)

find_library(ONNXRUNTIME_LIBRARIES
    NAMES onnxruntime
    PATHS
        ${ONNXRUNTIME_ROOT_PATH}/lib
        /usr/lib
        /usr/local/lib
        /opt/onnxruntime/lib
)

if(ONNXRUNTIME_INCLUDE_DIRS AND ONNXRUNTIME_LIBRARIES)
    message(STATUS "✓ Found ONNX Runtime")
    message(STATUS "  Include: ${ONNXRUNTIME_INCLUDE_DIRS}")
    message(STATUS "  Library: ${ONNXRUNTIME_LIBRARIES}")
    set(ONNXRUNTIME_FOUND TRUE)
else()
    message(WARNING "✗ ONNX Runtime not found. YOLOv5 detection will be disabled.")
    message(STATUS "  To install ONNX Runtime:")
    message(STATUS "  - Download from: https://github.com/microsoft/onnxruntime/releases")
    message(STATUS "  - Or: sudo apt-get install libonnxruntime-dev (Ubuntu)")
    set(ONNXRUNTIME_FOUND FALSE)
endif()

# Autonomous Rover (Camera -> Robot Bridge)
set(AUTONOMOUS_ROVER_SOURCES
    src/autonomous_rover.cpp
    src/robot_bridge.cpp
    src/serial_transport.cpp
    src/path_planner.cpp
    src/obstacle_detector.cpp
)

# Add YOLO detector if ONNX Runtime is available
if(ONNXRUNTIME_FOUND)
    list(APPEND AUTONOMOUS_ROVER_SOURCES src/yolo_detector.cpp)
    add_definitions(-DUSE_YOLO_DETECTOR)
    message(STATUS "✓ YOLOv5 detector enabled")
else()
    message(STATUS "⚠ YOLOv5 detector disabled (ONNX Runtime not found)")
endif()

add_executable(autonomous_rover ${AUTONOMOUS_ROVER_SOURCES})
target_link_libraries(autonomous_rover ${OpenCV_LIBS})

if(ESPEAK_INCLUDE_DIR)
    target_include_directories(autonomous_rover PRIVATE ${ESPEAK_INCLUDE_DIR})
endif()
if(ESPEAK_LIBRARY)
    target_link_libraries(autonomous_rover ${ESPEAK_LIBRARY})
endif()

if(ONNXRUNTIME_FOUND)
    target_include_directories(autonomous_rover PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
    target_link_libraries(autonomous_rover ${ONNXRUNTIME_LIBRARIES})
endif()
