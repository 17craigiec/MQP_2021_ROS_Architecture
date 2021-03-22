#include "./public/UART.h"
#include <iostream>

int main(int argc, char const *argv[])
{
    UART my_uart("/dev/ttyAMA0");

    while(1)
    {
        cout << "Line From Thread:  " << my_uart.returnCurrentLine() << endl;
    }
    
    return 0;
}
