#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/normalVectorTrigger.h"
#include "drone_architecture/normalVectorStatus.h"
#include "drone_architecture/pixhawkControl.h"
#include "drone_architecture/pixhawkControlStatus.h"
#include "drone_architecture/endEffectorControl.h"
#include "drone_architecture/flowerData.h"

#include <iostream>

using namespace std;

enum Option
{
  searchForFlower, lockOnFlower, centerFlower, allignWithFlower, confirmAllignedWithFlower, allignServo,
  lowerToFlower, pollinateFlower, returnToSearchHeight, resetServoPosition, 
};

int  choice= searchForFlower;
int nextChoice = searchForFlower;
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
            // TODO: May Need to Reset Normal Vector Trigger 
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

void State_Machine(ros::Publisher CalculateNormalVector_pub, ros::Publisher PixhawkMessage_pub, ros::Publisher EndEffector_pub )
{
    switch (choice)
    {
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
            drone_architecture::pixhawkControl pixhawk_msg;
            pixhawk_msg.desiredState = "centerOnFlower";
            pixhawk_msg.m_flower = targetFlower;
            PixhawkMessage_pub.publish(pixhawk_msg);
            choice = centerFlower;
            nextChoice = allignWithFlower;
        }
        break;

        case allignWithFlower:
        {
            // drone_architecture::normalVectorTrigger normalVector_msg;
            // normalVector_msg.start = true;
            // normalVector_msg.flower = flowerDataMessage.flowerArray[0];
            // CalculateNormalVector_pub.publish(normalVector_msg);
            // cout << "allignWithFlower Case" << endl;
            // choice = confirmAllignedWithFlower;
        }
        break;

        case confirmAllignedWithFlower:
        {
            cout << "Waiting for Confirmation" << endl;
            choice = confirmAllignedWithFlower;
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
    ros::Publisher PixhawkMessage_pub = n.advertise<drone_architecture::pixhawkControl>("pixhawkControllerState", 1000);
    ros::Publisher EndEffector_pub = n.advertise<drone_architecture::endEffectorControl>("EndEffectorControl", 1000);

    
    ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1000, flowerCallback);
    ros::Subscriber Pixhawk_sub = n.subscribe("pixhawkControlStatus", 1000, pixhawkCallback);
    ros::Subscriber NormalVector_sub = n.subscribe("normalVectorStatus", 1000, NormalVectorCallback);



    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        State_Machine(CalculateNormalVector_pub, PixhawkMessage_pub, EndEffector_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
