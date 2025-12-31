#pragma once

#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

class SerialPort
{
public:
    struct PortInfo
    {
        std::string port_name;
        std::uint16_t vid{};
        std::uint16_t pid{};
    };

    static auto ListPorts() -> std::vector<PortInfo>;

    explicit SerialPort(std::string_view portName);
    virtual ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort(SerialPort&&) = delete;
    auto operator=(const SerialPort&) = delete;
    auto operator=(SerialPort&&) = delete;

    void Open() noexcept;
    void Close() noexcept;

    bool Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept;

    void Write(std::string_view data);
    std::string Read() const;

    void SetBaudRate(std::size_t baudRate) noexcept;

    [[nodiscard]] bool IsOpen() const noexcept;


private:
    class SerialPortImpl;
    std::unique_ptr<SerialPortImpl> impl;
};