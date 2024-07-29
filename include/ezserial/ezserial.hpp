#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>
#include <string_view>

class SerialPort
{
public:
    explicit SerialPort(std::string_view portName);
    virtual ~SerialPort();

    void Open() noexcept;
    void Close() noexcept;

    bool Poll(const std::optional<std::chrono::milliseconds>& timeout) noexcept;

    void Write(std::string_view data);
    std::string Read();

    void SetBaudRate(std::size_t baudRate) noexcept;

    bool IsOpen() const noexcept;


private:
    class SerialPortImpl;
    std::unique_ptr<SerialPortImpl> impl;
};