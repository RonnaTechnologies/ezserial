#include <ezserial/ezserial.hpp>

#include <Windows.h>

#include <iostream>
#include <string>


class SerialPort::SerialPortImpl
{
public:

    explicit SerialPortImpl(std::string_view port) 
    : portName{"\\\\.\\" + std::string{port}}
    {
    }

    ~SerialPortImpl()
    {

    }

    void Open() noexcept
    {

        hCom = CreateFile(portName.c_str(), // port name
        GENERIC_READ | GENERIC_WRITE, // Read/Write
        0,                            // No Sharing
        NULL,                         // No Security
        OPEN_EXISTING,                // Open existing port only
        0,                            // Non Overlapped I/O
        NULL);                        // Null for Comm Devices

        if(hCom==INVALID_HANDLE_VALUE)
        {
            if(GetLastError()==ERROR_FILE_NOT_FOUND)
            {
                std::cout << "Serial port doesn't exist." << std::endl;
            }
            std::cout << "Unknown error..." << std::endl;
        }

        DCB serialParams = { 0 };
        GetCommState(hCom, &serialParams);
        serialParams.fBinary = TRUE;
        serialParams.fDtrControl = DTR_CONTROL_ENABLE;
        serialParams.fDsrSensitivity = FALSE;
        serialParams.fTXContinueOnXoff = FALSE;
        serialParams.fOutX = FALSE;
        serialParams.fInX = FALSE;
        serialParams.fErrorChar = FALSE;
        serialParams.fNull = FALSE;
        serialParams.fRtsControl = RTS_CONTROL_ENABLE;
        serialParams.fAbortOnError = FALSE;
        serialParams.fOutxCtsFlow = FALSE;
        serialParams.fOutxDsrFlow = FALSE;
        serialParams.DCBlength = sizeof(serialParams);

        serialParams.BaudRate = 9600;
        serialParams.ByteSize = 8;
        serialParams.StopBits = ONESTOPBIT;
        serialParams.Parity = NOPARITY;

        SetCommState(hCom, &serialParams);
        COMMTIMEOUTS timeout = { 0 };
        timeout.ReadIntervalTimeout = 50;
        timeout.ReadTotalTimeoutConstant = 50;
        timeout.ReadTotalTimeoutMultiplier = 10;
        timeout.WriteTotalTimeoutConstant = 50;
        timeout.WriteTotalTimeoutMultiplier = 10;

        SetCommTimeouts(hCom, &timeout);
    }

    void Close() noexcept
    {
        if(hCom == nullptr)
        {
            return;
        }

        CloseHandle(hCom);
        hCom = nullptr;
    }

    bool IsOpen() const noexcept
    {
        return hCom != nullptr;
    }

    void SetBaudRate(std::size_t b)
    {

    }

    std::string Read()
    {
        constexpr auto BUFFERLENGTH = 4095;
        char szBuff[BUFFERLENGTH + 1] = {0};
        DWORD dwBytesRead = 0;
        if(!ReadFile(hCom, szBuff, BUFFERLENGTH, &dwBytesRead, NULL))
        {
            //error occurred. Report to user.
        }
        return std::string{&szBuff[0]};
    }

private:

    HANDLE hCom = nullptr;
    std::string portName;

};


SerialPort::SerialPort(std::string_view portName)
: impl{std::make_unique<SerialPortImpl>(portName)}
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
