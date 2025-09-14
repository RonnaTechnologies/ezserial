#include <ezserial/ezserial.hpp>

#include <algorithm>
#include <array>
#include <cstdio> // Standard input / output functions
#include <cstdlib>
#include <cstring> // String function definitions
#include <filesystem>
#include <iostream>
#include <string>
#include <unordered_map>

#include <cerrno>    // Error number definitions
#include <fcntl.h>   // File control definitions
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard function definitions

class SerialPort::SerialPortImpl
{

public:
    explicit SerialPortImpl(std::string_view port) : portName{ port }
    {
    }

    ~SerialPortImpl()
    {
    }

    void Open()
    {
        handle = ::open(portName.c_str(), O_RDWR | O_NOCTTY);
        if (handle < 0)
        {
            return;
        }

        termios tty;
        if (tcgetattr(handle, &tty) != 0)
        {
            ::close(handle);
            return;
        }

        const auto baudRateIt = baudRates.find(baudRate);
        if (baudRateIt == baudRates.cend())
        {
            ::close(handle);
            return;
        }

        const auto rate = baudRateIt->second;
        cfsetospeed(&tty, rate);
        cfsetispeed(&tty, rate);

        tty.c_cflag &= ~PARENB;                         // No parity
        tty.c_cflag &= ~CSTOPB;                         // 1 stop bit
        tty.c_cflag &= ~CSIZE;                          // Clear size bits
        tty.c_cflag |= CS8;                             // 8 bits per byte
        tty.c_cflag &= ~CRTSCTS;                        // No flow control
        tty.c_cflag |= CREAD | CLOCAL;                  // Enable receiver, ignore control lines
        tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        tty.c_oflag &= ~OPOST;                          // Raw output
        tty.c_cc[VMIN] = 0;                             // Read doesn't block
        tty.c_cc[VTIME] = 0;                            // No inter-character timer

        cfmakeraw(&tty);
        tcflush(handle, TCIFLUSH);

        if (tcsetattr(handle, TCSANOW, &tty) != 0)
        {
            ::close(handle);
            return;
        }

        isOpen = true;
        return;
    }


    void Close() noexcept
    {
        if (isOpen && handle != -1)
        {
            ::close(handle);
            isOpen = false;
            handle = -1;
        }
    }

    void Write(std::string_view data)
    {
        if (!isOpen)
        {
            return;
        }
        ::write(handle, data.data(), data.size());
    }

    std::string Read()
    {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(handle, &readfds);

        struct timeval timeout;
        timeout.tv_sec = 0;       // 0 seconds
        timeout.tv_usec = 100000; // 100ms timeout

        int ready = select(handle + 1, &readfds, nullptr, nullptr, &timeout);
        if (ready < 0)
        {
            // Error in select
            return {};
        }
        else if (ready == 0)
        {
            // Timeout: no data available
            return {};
        }

        // Data is available to read
        std::array<char, 257> smallBuff = { 0 };
        const auto n = ::read(handle, smallBuff.data(), smallBuff.size() - 1);
        if (n < 0)
        {
            // Error in read
            return {};
        }
        else if (n == 0)
        {
            // EOF or no data
            return {};
        }

        smallBuff[n] = '\0';
        return std::string{ smallBuff.data(), static_cast<std::size_t>(n) };
    }


    void SetBaudRate(std::size_t baudRate) noexcept
    {
        this->baudRate = baudRate;
    }

    bool IsOpen() const noexcept
    {
        return isOpen;
    }

private:
    std::string portName;
    bool isOpen{ false };
    std::size_t baudRate;
    int handle{};

    inline static std::unordered_map<std::size_t, speed_t> baudRates{
        { 0UL, B0 },
        { 50UL, B50 },
        { 75UL, B75 },
        { 110UL, B110 },
        { 134UL, B134 },
        { 150UL, B150 },
        { 200UL, B200 },
        { 300UL, B300 },
        { 600UL, B600 },
        { 1200UL, B1200 },
        { 1800UL, B1800 },
        { 2400UL, B2400 },
        { 4800UL, B4800 },
        { 9600UL, B9600 },
        { 19200UL, B19200 },
        { 38400UL, B38400 },
        { 57600UL, B57600 },
        { 115200UL, B115200 },
        { 230400UL, B230400 },
        { 460800UL, B460800 },
        { 500000UL, B500000 },
        { 576000UL, B576000 },
        { 921600UL, B921600 },
        { 1000000UL, B1000000 },
        { 1152000UL, B1152000 },
        { 1500000UL, B1500000 },
        { 2000000UL, B2000000 },
        { 2500000UL, B2500000 },
        { 3000000UL, B3000000 },
        { 3500000UL, B3500000 },
        { 4000000UL, B4000000 },
    };
};

SerialPort::SerialPort(std::string_view portName) : impl{ std::make_unique<SerialPortImpl>(portName) }
{
}

SerialPort::~SerialPort()
{
}

void SerialPort::Open() noexcept
{
    impl->Open();
}

void SerialPort::Close() noexcept
{
    impl->Close();
}

void SerialPort::Write(std::string_view data)
{
    impl->Write(data);
}

std::string SerialPort::Read()
{
    return impl->Read();
}

void SerialPort::SetBaudRate(std::size_t baudRate) noexcept
{
    impl->SetBaudRate(baudRate);
}

bool SerialPort::IsOpen() const noexcept
{
    return impl->IsOpen();
}
