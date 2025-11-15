#include "serial_transport.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <errno.h>
#include <string.h>
#include <iostream>

SerialTransport::SerialTransport() : fd(-1) {}
SerialTransport::~SerialTransport() { close(); }

static speed_t baudToFlag(int baud)
{
    switch (baud)
    {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B115200;
    }
}

bool SerialTransport::configurePort(int baudRate)
{
    struct termios tio;
    if (tcgetattr(fd, &tio) != 0)
    {
        std::cerr << "tcgetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS; // no HW flow
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 0;   // non-blocking read
    tio.c_cc[VTIME] = 0;  // no inter-byte timer

    speed_t sp = baudToFlag(baudRate);
    cfsetispeed(&tio, sp);
    cfsetospeed(&tio, sp);

    if (tcsetattr(fd, TCSANOW, &tio) != 0)
    {
        std::cerr << "tcsetattr failed: " << strerror(errno) << "\n";
        return false;
    }

    return true;
}

bool SerialTransport::open(const std::string &devicePath, int baudRate)
{
    close();
    fd = ::open(devicePath.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0)
    {
        std::cerr << "open(" << devicePath << ") failed: " << strerror(errno) << "\n";
        return false;
    }
    if (!configurePort(baudRate))
    {
        close();
        return false;
    }
    return true;
}

void SerialTransport::close()
{
    if (fd >= 0)
    {
        ::close(fd);
        fd = -1;
    }
}

bool SerialTransport::isOpen() const
{
    return fd >= 0;
}

bool SerialTransport::sendLine(const std::string &line)
{
    if (fd < 0) return false;
    std::string data = line;
    if (data.empty() || data.back() != '\n') data.push_back('\n');
    ssize_t n = ::write(fd, data.data(), data.size());
    return n == (ssize_t)data.size();
}

bool SerialTransport::sendBytes(const void* data, size_t len)
{
    if (fd < 0) return false;
    if (data == nullptr || len == 0) return false;
    ssize_t n = ::write(fd, data, len);
    return n == (ssize_t)len;
}

bool SerialTransport::sendByte(char c)
{
    return sendBytes(&c, 1);
}


