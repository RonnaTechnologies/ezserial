
#include <ezserial/ezserial.hpp>

#include <fcntl.h> // File control definitions
#include <fstream>
#include <sys/epoll.h>
#include <termios.h> // POSIX terminal control definitions
#include <unistd.h>  // UNIX standard function definitions

#include <algorithm>
#include <array>
#include <cstdio> // Standard input / output functions
#include <cstdlib>
#include <cstring> // String function definitions
#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>


class SerialPort::SerialPortImpl
{

public:
    explicit SerialPortImpl(std::string_view port) : portName{ port }
    {
    }

    ~SerialPortImpl() = default;

    void Open() noexcept
    {
        handle = ::open(portName.c_str(), O_RDWR | O_NOCTTY);
        termios tty{};

        if (tcgetattr(handle, &tty) != 0)
        {
            return;
        }

        const auto baudRateIt = baudRates.find(baudRate);
        if (baudRateIt == baudRates.cend())
        {
            return;
        }

        const auto rate = baudRateIt->second;

        // Set baud rate
        cfsetospeed(&tty, rate);
        cfsetispeed(&tty, rate);

        tty.c_cflag &= ~PARENB; // Make 8n1 (Disable parity)
        tty.c_cflag &= ~CSTOPB; // Use a single stop bit
        tty.c_cflag &= ~CSIZE;  // Clear all the size bits
        tty.c_cflag |= CS8;     // 8 bits per byte

        tty.c_cflag &= ~CRTSCTS;       // Disable flow control
        tty.c_cc[VMIN] = 0;            // Read blocks
        tty.c_cc[VTIME] = 1;           // 1 second read timeout
        tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines

        // Raw mode
        cfmakeraw(&tty);

        // Flush Port, then applies attributes
        tcflush(this->handle, TCIFLUSH);

        if (tcsetattr(this->handle, TCSANOW, &tty) != 0)
        {
            return;
        }

        isOpen = true;
    }

    void Close() noexcept
    {
        if (isOpen && handle != -1)
        {
            ::close(handle);
            isOpen = false;
            if (epoll_handle != -1)
            {
                ::close(epoll_handle);
                epoll_handle = -1;
            }
            handle = -1;
        }
    }

    bool Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept
    {
        std::array<::epoll_event, 1U> e{};
        if (epoll_handle == -1)
        {
            epoll_handle = ::epoll_create1(0);
            e.front().events |= ::EPOLLRDNORM;

            ::epoll_ctl(epoll_handle, EPOLL_CTL_ADD, handle, e.data());
        }

        e.front().events = 0;
        const auto timeout_ms = timeout.has_value() ? timeout->count() : -1;
        ::epoll_wait(epoll_handle, e.data(), e.size(), timeout_ms);

        return e.front().events | ::EPOLLRDNORM;
    }

    void Write(std::string_view data)
    {
        ::write(handle, data.data(), data.size());
    }

    std::string Read() const
    {
        static constexpr auto smallBuffSize = 256U;
        std::array<char, smallBuffSize> smallBuff{ 0 };
        ::read(handle, smallBuff.data(), smallBuff.size());

        return std::string{ smallBuff.data() };
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
    std::size_t baudRate{};
    int handle{};
    int epoll_handle{ -1 };

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

auto SerialPort::ListPorts() -> std::vector<SerialPort::PortInfo>
{
    std::vector<SerialPort::PortInfo> ports;

    const auto tty_path = std::filesystem::path{ "/sys/class/tty" };

    static constexpr auto allowed_devices =
    std::array{ "ttyS", "ttyUSB", "ttyXRUSB", "ttyACM", "ttyAMA", "rfcomm", "ttyAP", "ttyGS" };

    std::vector<std::string> ports_names;
    for (const auto& entry : std::filesystem::directory_iterator{ tty_path })
    {
        const auto port_name = entry.path().filename().generic_string();
        if (entry.is_directory() &&
            std::ranges::any_of(allowed_devices,
                                [&](const char* name) { return port_name.find(name) != std::string::npos; }))
        {

            ports_names.push_back(port_name);
        }
    }

    for (const auto& port_name : ports_names)
    {
        if (!std::filesystem::exists(tty_path / port_name / "device"))
        {
            continue;
        }

        const auto device_path = std::filesystem::canonical(tty_path / port_name / "device");
        const auto subsystem = std::filesystem::canonical(device_path / "subsystem").filename();
        const auto usb_interface_path = [](const std::string& subsystem,
                                           const std::filesystem::path& device_path) -> std::optional<std::filesystem::path>
        {
            if (subsystem == "usb")
            {
                return device_path;
            }
            if (subsystem == "usb-serial")
            {
                return device_path.parent_path();
            }

            return std::nullopt;
        }(subsystem, device_path);

        if (usb_interface_path.has_value())
        {
            const auto usb_device_path = usb_interface_path->parent_path();
            const auto read_file_content = [](const std::filesystem::path& file_path)
            {
                auto file = std::ifstream{ file_path };
                std::string content;
                file >> content;
                return content;
            };

            try
            {
                const auto vid = read_file_content(usb_device_path / "idVendor");
                const auto pid = read_file_content(usb_device_path / "idProduct");
                const auto port_info = SerialPort::PortInfo{ .port_name = "/dev/" + port_name,
                                                             .vid = static_cast<std::uint16_t>(std::stoi(vid)),
                                                             .pid = static_cast<std::uint16_t>(std::stoi(pid)) };
                ports.emplace_back(port_info);
            }
            catch (...)
            {
                continue;
            }
        }
    }
    return ports;
}


SerialPort::SerialPort(std::string_view portName) : impl{ std::make_unique<SerialPortImpl>(portName) }
{
}

SerialPort::~SerialPort() = default;

void SerialPort::Open() noexcept
{
    impl->Open();
}

void SerialPort::Close() noexcept
{
    impl->Close();
}

bool SerialPort::Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept
{
    return impl->Poll(timeout);
}

void SerialPort::Write(std::string_view data)
{
    impl->Write(data);
}

std::string SerialPort::Read() const
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
