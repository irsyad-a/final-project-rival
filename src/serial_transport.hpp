#pragma once
#include <string>

// Simple POSIX serial transport (USB or Bluetooth RFCOMM).
// Open with a device path like /dev/ttyUSB0, /dev/ttyACM0, or /dev/rfcomm0
class SerialTransport
{
public:
    SerialTransport();
    ~SerialTransport();

    bool open(const std::string &devicePath, int baudRate);
    void close();
    bool isOpen() const;
    bool sendLine(const std::string &line); // appends '\n' if missing
    bool sendByte(char c);                  // send single byte, no newline
    bool sendBytes(const void* data, size_t len); // send raw bytes

private:
    int fd;
    bool configurePort(int baudRate);
};


