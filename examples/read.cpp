#include <ezserial/ezserial.hpp>

#include <iostream>

int main()
{
    SerialPort sp{"/dev/ttyACM0"};
    sp.SetBaudRate(9'600);
    sp.Open();
    for(std::size_t i = 0; i < 10; ++i)
    {
        std::cout << sp.Read() << std::endl;
    }
    sp.Close();
}