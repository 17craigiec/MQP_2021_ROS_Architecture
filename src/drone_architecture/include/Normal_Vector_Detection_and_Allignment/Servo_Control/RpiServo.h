#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>

#include <pigpio.h>

using namespace std;


class RpiServo
{
private:
    int m_servo_pin = -1;

public:
    int m_setpoint = 0;

    RpiServo(int servo_pin);
    ~RpiServo();

    void setServo(int setpoint);
};


