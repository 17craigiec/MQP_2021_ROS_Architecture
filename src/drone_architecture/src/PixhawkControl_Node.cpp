#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/pixhawkControl.h"
#include "drone_architecture/pixhawkControlStatus.h"
#include <iostream>

using namespace std;

enum Option
{
  waitForCommand, centerOnFlower, LowerToFlower, FlyTo,
};

int  state= waitForCommand;
drone_architecture::flower targetFlower;

int confirmState(string desiredState)
{
    if (desiredState == "centerOnFlower") return centerOnFlower;
    else if (desiredState == "LowerToFlower") return LowerToFlower; 
    else if (desiredState == "FlyTo") return FlyTo; 
    else return waitForCommand; 

}

void UpdateStateCallback(const drone_architecture::pixhawkControl& msg)
{
    state = confirmState(msg.desiredState);
    targetFlower = msg.m_flower;
    std::cout<<"Received Message in controller"<<endl;
}

void State_Machine()
{
    switch (state)
    {
        case waitForCommand:
        {
            cout << "waitForCommand Case" << endl;
            state = waitForCommand;
        }
        break;

        case centerOnFlower:
        {
            //TODO: Design a PID controller to center flower centroid to frame centroid
            cout << "centerOnFlower Case" << endl;
            state = centerOnFlower;
        }
        break;

        default:
        {
            cout << "INVALID INPUT" << endl;
        }
        break;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "PixhawkControl_Node");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("pixhawkControllerState", 1000, UpdateStateCallback);


    ros::Publisher pixhawkControllerStatus_pub = n.advertise<drone_architecture::pixhawkControlStatus>("pixhawkControlStatus", 1000);
    
    
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 1000);
    

    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::TwistStamped>("mavros/local_position/velocity", 1000);



    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        State_Machine();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
