#include "RpiServo.h"

RpiServo::RpiServo(int servo_pin)
{
    m_servo_pin = servo_pin;
}

RpiServo::~RpiServo()
{
    gpioTerminate();
}

void RpiServo::setServo(int setpoint)
{
    m_setpoint = setpoint;

    if(m_setpoint > 1000)
    {
        m_setpoint = 1000;
        cout << "Error, Servo setpoint is out of bounds: " << m_setpoint << endl;
    }else if(m_setpoint < 0)
    {
        m_setpoint = 0;
        cout << "Error, Servo setpoint is out of bounds: " << m_setpoint << endl;
    }
    
    gpioServo(m_servo_pin, m_setpoint+1000);
}