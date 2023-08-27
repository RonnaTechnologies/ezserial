#include <ezserial/ezserial.hpp>

#include <iostream>
#include <span>

int main(int argc, const char** const argv)
{
    const auto args = std::span(argv, argc);

    if(args.size() !=  2)
    {
        return EXIT_FAILURE;
    }

    const auto portName = args.back();

    SerialPort sp{portName};
    sp.SetBaudRate(115'200);
    sp.Open();
    for(std::size_t i = 0; i < 10; ++i)
    {
        std::cout << sp.Read() << std::endl;
    }
    sp.Close();

    return EXIT_SUCCESS;
}