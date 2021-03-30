#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/endEffectorControl.h"

#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <pigpio.h>

using namespace std;

void endEffectorCallback(const drone_architecture::endEffectorControl& msg)
{
    int setPoint = msg.m_setpoint;
    if(setPoint > 1000)
    {
        setPoint = 1000;
        cout << "Error, Servo setpoint is out of bounds: " << setPoint << endl;
    }else if(setPoint < 0)
    {
        setPoint = 0;
        cout << "Error, Servo setpoint is out of bounds: " << setPoint << endl;
    }
    gpioServo(msg.m_pinNumber, setPoint+1000);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "EndEffector_Node");
    ros::NodeHandle n;
    
    ros::Subscriber EndEffector_sub = n.subscribe("EndEffectorControl", 1000, endEffectorCallback);

    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
