#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/pixhawkInterface.h"
#include "drone_architecture/pixhawkControlStatus.h"
#include "drone_architecture/endEffectorControl.h"
#include "drone_architecture/flowerData.h"
#include "PID_Driver.cpp"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <unistd.h>
#include <math.h>  
#include <sys/time.h>
#define PI 3.14159265

using namespace std;

enum Option
{
  sendPositionalMessages, offBoard, arming, initStateMachine, searchForFlower, cyclical_search, lockOnFlower, centerFlower, allignServo,
  lowerToFlower, pollinateFlower, returnToSearchHeight, resetServoPosition, 
};

int  choice= initStateMachine;
drone_architecture::flowerData flowerDataMessage;
drone_architecture::flower targetFlower;
int desiredID = 0;
PID_Driver pid_velocity_x(.01, 0, 0);
PID_Driver pid_velocity_y(.01, 0, 0);

// time_t start_search_time;
struct timeval tp;
double start_search_time;

void flowerCallback(const drone_architecture::flowerData& msg)
{
    cout << "see flower Case" << endl;
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

geometry_msgs::PoseStamped currentPosition;
geometry_msgs::PoseStamped droneOrigin;

void pixhawkCallback(const geometry_msgs::PoseStamped& msg)
{
    currentPosition = msg;
}

float compassHeading;
void compassCallback(const std_msgs::Float64& msg)
{
    compassHeading = msg.data;
}

void State_Machine(ros::Publisher EndEffector_pub, ros::Publisher pixHawk_pub)
{
    switch (choice)
    {
        case initStateMachine:
        {
            usleep(5000000); //wait for MAVROS
            drone_architecture::endEffectorControl endEffectorMessage; //reset servo
            endEffectorMessage.m_XCentroid = 0;
            endEffectorMessage.m_YCentroid = 110;
            endEffectorMessage.m_reset = true;
            EndEffector_pub.publish(endEffectorMessage);
            choice = searchForFlower; // sendPositionalMessages;
        }
        break;

        case sendPositionalMessages: 
        {
            cout << "sendPositionalMessages Case" << endl;
            drone_architecture::pixhawkInterface InterfaceMessage;
            geometry_msgs::Pose position_message;
            position_message.position.x = currentPosition.pose.position.x;
            position_message.position.y = currentPosition.pose.position.y;
            position_message.position.z = 1;

            InterfaceMessage.pose = position_message;
            InterfaceMessage.flight_command = "POSTITION";
            for(int i; i < 200; i++)
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
            droneOrigin = currentPosition;
            sleep(5);    
        }
        break;

        case arming:
        {
            cout << "arming Case" << endl;
            drone_architecture::pixhawkInterface InterfaceMessage;
            InterfaceMessage.flight_command = "ARM";
            pixHawk_pub.publish(InterfaceMessage);  
            choice = searchForFlower;
            sleep(5);      
        }
        break;

        case searchForFlower:
        {
            cout << "searchForFlower Case" << endl;
            // ====================================
            // Start timing the search for flower state
            // start_search_time = time(nullptr);
            gettimeofday(&tp, NULL);
            start_search_time = (double)tp.tv_sec + (double)tp.tv_usec / 1000000;

            // Choose some search algorithim state
            choice = cyclical_search;

            // ====================================
            // choice = searchForFlower;
        }
        break;

        case cyclical_search:
        {
            gettimeofday(&tp, NULL);
            double elapsed_time = (double)tp.tv_sec + (double)tp.tv_usec / 1000000 - start_search_time;
            // cout << "Elapsed_Time: " << elapsed_time << endl;

            // Cyclical Functions
            double v_max = 0.5;    // max velocity (meters/second)
            double r_max = 8;      // max search radius (meters)
            double a = 1;          // distance between overlapping cyclical paths (meters)

            double current_theta = sqrt(v_max*(elapsed_time/a));
            double curent_radius = a*current_theta/(2*PI);

            double pos_x = curent_radius*cos(current_theta);
            double pos_y = curent_radius*sin(current_theta);

            drone_architecture::pixhawkInterface InterfaceMessage;
            geometry_msgs::Pose position_message;

            position_message.position.x = droneOrigin.pose.position.x + pos_x;
            position_message.position.y = droneOrigin.pose.position.y + pos_y;
            position_message.position.z = droneOrigin.pose.position.z+1;
            position_message.orientation = droneOrigin.pose.orientation;
            InterfaceMessage.pose = position_message;
            InterfaceMessage.flight_command = "POSITION";

            pixHawk_pub.publish(InterfaceMessage);
            cout << pos_x << ", " << pos_y << endl;

        }
        break;

        case lockOnFlower:
        {
            int numberOfFlowers = flowerDataMessage.flowerArray.size();
            desiredID = rand() % numberOfFlowers;
            cout << "lockOnFlower Case" << endl;
            choice = allignServo; //centerFlower;
        }
        break;

        case centerFlower:
        {
            int setpointX = 160;
            int setpointY = 120;
            float x_velocity = pid_velocity_x.setVelocity(setpointX, targetFlower.m_X);
            float y_velocity = pid_velocity_y.setVelocity(setpointY, targetFlower.m_Y);

            drone_architecture::pixhawkInterface InterfaceMessage;
            geometry_msgs::Twist velocity_message;

            velocity_message.linear.x = x_velocity * cos(compassHeading * (PI/180))-y_velocity * sin(compassHeading * (PI/180));
            velocity_message.linear.y = x_velocity * sin(compassHeading * (PI/180)) + y_velocity * cos(compassHeading * (PI/180));
            velocity_message.linear.z = 0;

            //std::cout<<"Velocities: "<<velocity_message.linear.x<<", "<< velocity_message.linear.y<<endl;

            if(velocity_message.linear.x > 1.5)
            {
                velocity_message.linear.x = 1.5;
            }
            if(velocity_message.linear.x < -1.5)
            {
                velocity_message.linear.x = -1.5;
            }
            if(velocity_message.linear.y > 1.5)
            {
                velocity_message.linear.y = 1.5;
            }
            if(velocity_message.linear.y < -1.5)
            {
                velocity_message.linear.y = -1.5;
            }

            //std::cout<<"Velocities: "<<velocity_message.linear.x<<", "<< velocity_message.linear.y<<endl;

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
            drone_architecture::endEffectorControl endEffectorMessage;
            endEffectorMessage.m_XCentroid = targetFlower.m_X;
            endEffectorMessage.m_YCentroid = targetFlower.m_Y;
            endEffectorMessage.m_reset = false;
            EndEffector_pub.publish(endEffectorMessage);
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
            // 50 is 0
            // 96 is 45
            // 143 is 90
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

    ros::Publisher EndEffector_pub = n.advertise<drone_architecture::endEffectorControl>("EndEffectorControl", 1);
    ros::Publisher pixHawk_pub = n.advertise<drone_architecture::pixhawkInterface>("/command", 1);

    
    ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1, flowerCallback);
    ros::Subscriber Pixhawk_sub = n.subscribe("mavros/local_position/pose", 1, pixhawkCallback);
    ros::Subscriber Compass_sub = n.subscribe("mavros/global_position/compass_hdg", 1, compassCallback);


    ros::Rate loop_rate(20);
    ros::spinOnce();

    while (ros::ok()) {
        State_Machine(EndEffector_pub, pixHawk_pub);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
