#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/pixhawkInterface.h"
#include "drone_architecture/PIDVelocity.h"
#include "PID_Driver.cpp"
#include <geometry_msgs/Twist.h>
#include <iostream>

using namespace std;
const int setpointX = 160;
const int setpointY = 120;
ros::Publisher pixHawk_pub;

void PID_Velocity_Callback(const drone_architecture::PIDVelocity& msg)
{
    PID_Driver pid_velocity_x(1, 0, 0);
    PID_Driver pid_velocity_y(1, 0, 0);
    float x_velocity = pid_velocity_x.setVelocity(setpointX, msg.m_flower.m_X);
    float y_velocity = pid_velocity_y.setVelocity(setpointY, msg.m_flower.m_Y);

    drone_architecture::pixhawkInterface InterfaceMessage;
    geometry_msgs::Twist velocity_message;
    velocity_message.linear.x = x_velocity;
    velocity_message.linear.y = y_velocity;
    velocity_message.linear.z = 0;

    std::cout<<"Velocities"<<endl;
    std::cout<<x_velocity<<endl;
    std::cout<<y_velocity<<endl;

    InterfaceMessage.twist = velocity_message;
    InterfaceMessage.flight_command = "VELOCITY";
    
    pixHawk_pub.publish(InterfaceMessage);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ControlLoop_Node");
    ros::NodeHandle n;
    pixHawk_pub= n.advertise<drone_architecture::pixhawkInterface>("/command", 1000);

    ros::Subscriber PIDVelocity_sub = n.subscribe("PIDVelocityController", 1000, PID_Velocity_Callback);

    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
