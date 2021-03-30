#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/normalVectorTrigger.h"
#include "drone_architecture/normalVectorStatus.h"
#include "drone_architecture/PIDVelocity.h"
#include "drone_architecture/pixhawkInterface.h"
#include "drone_architecture/pixhawkControlStatus.h"
#include "drone_architecture/endEffectorControl.h"
#include "drone_architecture/flowerData.h"


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
    std::cout<<"Received Message"<<endl;
}

void pixhawkCallback(const drone_architecture::pixhawkControlStatus& msg)
{
    choice = nextChoice;
}

void NormalVectorCallback(const drone_architecture::normalVectorStatus& msg)
{

}

void State_Machine(ros::Publisher CalculateNormalVector_pub, ros::Publisher VelocityPID_pub, ros::Publisher EndEffector_pub, ros::Publisher pixHawk_pub)
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
            drone_architecture::PIDVelocity PID_msg;
            PID_msg.m_flower = targetFlower;
            VelocityPID_pub.publish(PID_msg);
            choice = centerFlower;
            nextChoice = allignWithFlower;
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

    ros::Publisher CalculateNormalVector_pub = n.advertise<drone_architecture::normalVectorTrigger>("calculateNormalVector", 1000);
    ros::Publisher VelocityPID_pub = n.advertise<drone_architecture::PIDVelocity>("PIDVelocityController", 1000);
    ros::Publisher EndEffector_pub = n.advertise<drone_architecture::endEffectorControl>("EndEffectorControl", 1000);
    ros::Publisher pixHawk_pub = n.advertise<drone_architecture::pixhawkInterface>("/command", 1000);

    
    ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1000, flowerCallback);
    ros::Subscriber Pixhawk_sub = n.subscribe("pixhawkControlStatus", 1000, pixhawkCallback);
    ros::Subscriber NormalVector_sub = n.subscribe("normalVectorStatus", 1000, NormalVectorCallback);



    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        State_Machine(CalculateNormalVector_pub, VelocityPID_pub, EndEffector_pub, pixHawk_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
