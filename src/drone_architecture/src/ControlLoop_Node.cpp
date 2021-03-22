#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flowerData.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/normalVectorTrigger.h"
#include "drone_architecture/normalVectorStatus.h"
#include "drone_architecture/pixhawkControl.h"
#include "drone_architecture/pixhawkControlStatus.h"
#include "drone_architecture/endEffectorControl.h"

#include <iostream>

using namespace std;
void flowerCallback(const drone_architecture::flowerData& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ControlLoop_Node");
    ros::NodeHandle n;

    ros::Publisher CalculateNormalVector_pub = n.advertise<drone_architecture::normalVectorTrigger>("calculateNormalVector", 1000);
    
    ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1000, flowerCallback);

    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
