// #include <stdio.h>
// #include <stdlib.h>
// #include <signal.h>
// #include <iostream>

// #include <pigpio.h>

// using namespace std;


// int main(int argc, char const *argv[])
// {
//     if (gpioInitialise() < 0) return -1;

//     for(int i = 1000; i < 2000; i++)
//     {
//         gpioServo(4, i);
//         cout << i << endl;
//         time_sleep(0.1);
//     }

//     for(int i = 2000; i > 1000; i--)
//     {
//         gpioServo(4, i);
//         cout << i << endl;
//         time_sleep(0.1);
//     }
    
//     gpioTerminate();

//     return 0;
// }



//  ========================================================================================================

// #include "RpiServo.h"

// using namespace std;


// int main(int argc, char const *argv[])
// {
//     if (gpioInitialise() < 0) return -1;
//     RpiServo sv(4);

//     for(int i = 0; i < 1000; i++)
//     {
//         sv.setServo(i);
//         cout << i << endl;
//         time_sleep(0.1);
//     }

//     for(int i = 1000; i > 0; i--)
//     {
//         sv.setServo(i);
//         cout << i << endl;
//         time_sleep(0.1);
//     }

//     gpioTerminate();

//     return 0;
// }


//  ========================================================================================================

#include "../Normal_Vector/public/BoxInfo.h"
#include "../Normal_Vector/public/UART.h"
#include "RpiServo.h"

using namespace std;


bool run = true;

void stop(int signum)
{
   run = false;
}

int main(int argc, char const *argv[])
{
    if (gpioInitialise() < 0) return -1;
    RpiServo sv(4);
    BoxInfo bi;
    UART my_uart("/dev/ttyAMA0");

    int camera_pixel_midpoint = 125;

    // Testing this for now
    // gpioSetSignalFunc(SIGINT, stop);

    while(run)
    {
        bi = readBoxInfo(my_uart.returnCurrentLine());

        if(bi.y != -1)
        {
            if(bi.y < camera_pixel_midpoint)
            {
                sv.setServo(sv.m_setpoint+5);
            }else
            {
                sv.setServo(sv.m_setpoint-5);
            }
        }

        time_sleep(0.01);
    }
    
    cout << "\nCleaning up threads" << endl;

    my_uart.join();

    cout << "Terminatin GPIO readings" << endl;

    gpioTerminate();

    return 0;
}