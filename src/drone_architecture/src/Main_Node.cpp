#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/pixhawkInterface.h"
#include "drone_architecture/pixhawkControlStatus.h"
#include "drone_architecture/endEffectorControl.h"
#include "drone_architecture/flowerData.h"
#include "PID_Driver.cpp"
#include <geometry_msgs/Twist.h>

#include <iostream>

using namespace std;

enum Option
{
  sendPositionalMessages, offBoard, arming, searchForFlower, lockOnFlower, centerFlower, allignWithFlower, confirmAllignedWithFlower, allignServo,
  lowerToFlower, pollinateFlower, returnToSearchHeight, resetServoPosition, 
};

int  choice= sendPositionalMessages;
int nextChoice = sendPositionalMessages;
drone_architecture::flowerData flowerDataMessage;
drone_architecture::flower targetFlower;
int desiredID = 0;

void flowerCallback(const drone_architecture::flowerData& msg)
{
    if(choice == searchForFlower)
    {
        choice = lockOnFlower;
        flowerDataMessage = msg;
    }
    else
    {
        try
        {
            targetFlower = msg.flowerArray[desiredID];
        }
        catch(const std::exception& e) //Flower disapeared for some reason
        {
            choice = lockOnFlower;
            flowerDataMessage = msg;
        }
    }
}

void pixhawkCallback(const drone_architecture::pixhawkControlStatus& msg)
{
    choice = nextChoice;
}

void State_Machine(ros::Publisher EndEffector_pub, ros::Publisher pixHawk_pub)
{
    switch (choice)
    {
        case sendPositionalMessages: 
        {
            cout << "sendPositionalMessages Case" << endl;
            drone_architecture::pixhawkInterface InterfaceMessage;
            geometry_msgs::Pose position_message;
            position_message.position.x = 0;
            position_message.position.y = 0;
            position_message.position.z = 2;

            InterfaceMessage.pose = position_message;
            InterfaceMessage.flight_command = "POSTITION";
            for(int i; i < 20; i++)
            {
                pixHawk_pub.publish(InterfaceMessage);
            }
            choice = offBoard;
        }
        break;

        case offBoard:
        {
            cout << "offBoard Case" << endl;
            drone_architecture::pixhawkInterface InterfaceMessage;
            InterfaceMessage.flight_command = "OFFBOARD";
            pixHawk_pub.publish(InterfaceMessage);   
            choice = arming;      
        }
        break;

        case arming:
        {
            cout << "arming Case" << endl;
            drone_architecture::pixhawkInterface InterfaceMessage;
            InterfaceMessage.flight_command = "ARM";
            pixHawk_pub.publish(InterfaceMessage);  
            choice = searchForFlower;      
        }
        break;

        case searchForFlower:
        {
            cout << "searchForFlower Case" << endl;
            choice = searchForFlower;
        }
        break;

        case lockOnFlower:
        {
            int numberOfFlowers = flowerDataMessage.flowerArray.size();
            desiredID = rand() % numberOfFlowers;
            cout << "lockOnFlower Case" << endl;
            choice = centerFlower;
        }
        break;

        case centerFlower:
        {
            int setpointX = 160;
            int setpointY = 120;
            PID_Driver pid_velocity_x(.01, 0, 0);
            PID_Driver pid_velocity_y(.01, 0, 0);
            float x_velocity = pid_velocity_x.setVelocity(setpointX, targetFlower.m_X);
            float y_velocity = pid_velocity_y.setVelocity(setpointY, targetFlower.m_Y);

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

            if(x_velocity==0 && y_velocity==0)
            {
                choice = allignServo;
            }
            else
            {
                choice = centerFlower;
            }
        }
        break;
        
        case allignServo:
        {
            cout << "allignServo Case" << endl;
            choice = allignServo;
        }
        break;

        case lowerToFlower:
        {
            cout << "lowerToFlower Case" << endl;
            choice = lowerToFlower;
        }
        break;

        case pollinateFlower:
        {
            cout << "pollinateFlower Case" << endl;
            choice = pollinateFlower;
        }
        break;

        case returnToSearchHeight:
        {
            cout << "returnToSearchHeight Case" << endl;
            choice = returnToSearchHeight;
        } 
        break;

        case resetServoPosition:
        {
            cout << "resetServoPosition Case" << endl;
            choice = resetServoPosition;
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
    ros::init(argc, argv, "Main_Node");
    ros::NodeHandle n;

    ros::Publisher EndEffector_pub = n.advertise<drone_architecture::endEffectorControl>("EndEffectorControl", 1000);
    ros::Publisher pixHawk_pub = n.advertise<drone_architecture::pixhawkInterface>("/command", 1000);

    
    ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1000, flowerCallback);
    ros::Subscriber Pixhawk_sub = n.subscribe("pixhawkControlStatus", 1000, pixhawkCallback);



    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        State_Machine(EndEffector_pub, pixHawk_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
