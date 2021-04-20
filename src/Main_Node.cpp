#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/pixhawkInterface.h"
#include "drone_architecture/endEffectorControl.h"
#include "drone_architecture/flowerData.h"
#include "drone_architecture/initializeSearch.h"
#include "drone_architecture/nextWaypoint.h"
#include "PID_Driver.cpp"
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/CommandHome.h>
#include "mavros_msgs/State.h"
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoint.h>
#include <iostream>
#include <unistd.h>
#include <math.h>  
#include <sys/time.h>
#define PI 3.14159265

using namespace std;

enum Option
{
  sendPositionalMessages, offBoard, arming, initStateMachine, searchForFlower, cyclical_search, orient_north, lockOnFlower, centerFlower, allignServo,
  lowerToFlower, pollinateFlower, returnToSearchHeight, resetServoPosition, updateHome
};

int choice = initStateMachine;
drone_architecture::flowerData flowerDataMessage;
drone_architecture::flower targetFlower;
int desiredID = 0;
PID_Driver pid_velocity_x(.01, 0, 0);
PID_Driver pid_velocity_y(.01, 0, 0);

// globals used in search path generation
// struct timeval tp;
double start_search_time;
double search_pattern_freq = 5;
// globals used for velocity setpoints
geometry_msgs::Pose lastSetpoint;
geometry_msgs::Pose northOrientation;

double getTime()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return (double)tp.tv_sec + (double)tp.tv_usec / 1000000;
}

void flowerCallback(const drone_architecture::flowerData& msg)
{
    // cout << "see flower Case" << endl;
    // if(choice == searchForFlower)
    // {
    //     choice = lockOnFlower;
    //     flowerDataMessage = msg;
    // }
    // else
    // {
    //     try
    //     {
    //         targetFlower = msg.flowerArray[desiredID];
    //     }
    //     catch(const std::exception& e) //Flower disapeared for some reason
    //     {
    //         choice = lockOnFlower;
    //         flowerDataMessage = msg;
    //     }
    // }
    if(choice == cyclical_search)
    {
        cout << "!!!!!!!!!!!!!!! FLOWER DETECTED !!!!!!!!!!!!!!!!" << endl;
        choice = lockOnFlower;
    }
}

geometry_msgs::PoseStamped currentPosition;
sensor_msgs::NavSatFix globalPosition;
geometry_msgs::PoseStamped droneOrigin;
mavros_msgs::HomePosition home_pose;
bool have_home{false};
mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

void pixhawkCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    currentPosition = *msg;
}

void global_pose_cb(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    globalPosition = *msg;
}

void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg)
{
    have_home = true;
    home_pose = *msg;
}

float compassHeading;
void compassCallback(const std_msgs::Float64& msg)
{
    compassHeading = msg.data;
}

void State_Machine(ros::Publisher EndEffector_pub, ros::Publisher pixHawk_pub, ros::Publisher search_pub, ros::ServiceClient next_waypoint_client, ros::ServiceClient update_home_client)
{
    switch (choice)
    {
        case initStateMachine:
        {
            // wait for FCU connection
            if(ros::ok() && current_state.connected){
                drone_architecture::endEffectorControl endEffectorMessage; //reset servo
                endEffectorMessage.m_XCentroid = 0;
                endEffectorMessage.m_YCentroid = 110;
                endEffectorMessage.m_reset = true;
                droneOrigin = currentPosition;
                EndEffector_pub.publish(endEffectorMessage);

                // Init north orientation
                northOrientation.orientation.x = -0.02;
                northOrientation.orientation.y = 0.007;
                northOrientation.orientation.z = -0.8139;
                northOrientation.orientation.w = -0.580;

                choice = centerFlower;
            }
        }
        break;

        case updateHome:
        {
            // wait for home position
            cout << "waiting for home position" << endl;
            if(ros::ok() && have_home){
                cout << "got home position" << endl;

                mavros_msgs::CommandHome newHome;
                newHome.request.latitude = globalPosition.latitude;
                newHome.request.longitude = globalPosition.longitude;
                newHome.request.altitude = globalPosition.altitude;      

                update_home_client.call(newHome);

                cout << "updated home position" << endl;
                choice = sendPositionalMessages;
            }
        }
        break;

        case sendPositionalMessages: 
        {
            cout << "sendPositionalMessages Case" << endl;
            drone_architecture::pixhawkInterface InterfaceMessage;
            geometry_msgs::Pose position_message;
            position_message.position.x = currentPosition.pose.position.x;
            position_message.position.y = currentPosition.pose.position.y;
            position_message.position.z = currentPosition.pose.position.z + 2;
            position_message.orientation = currentPosition.pose.orientation;

            InterfaceMessage.pose = position_message;
            InterfaceMessage.flight_command = "POSITION";
            for(int i=0; i < 10; i++)
            {
                pixHawk_pub.publish(InterfaceMessage);
            }

            // Record the positional setpoint message 
            lastSetpoint = position_message;
            choice = searchForFlower;
        }
        break;

        // case offBoard:
        // {
        //     cout << "offBoard Case" << endl;
        //     drone_architecture::pixhawkInterface InterfaceMessage;
        //     InterfaceMessage.flight_command = "OFFBOARD";
        //     pixHawk_pub.publish(InterfaceMessage);   
        //     choice = arming;  
        //     droneOrigin = currentPosition;
        //     sleep(5);    
        // }
        // break;

        // case arming:
        // {
        //     cout << "arming Case" << endl;
        //     drone_architecture::pixhawkInterface InterfaceMessage;
        //     InterfaceMessage.flight_command = "ARM";
        //     pixHawk_pub.publish(InterfaceMessage);  
        //     choice = searchForFlower;
        //     sleep(5);      
        // }
        // break;

        case searchForFlower:
        {
            // cout << "searchForFlower Case" << endl;
            // cout << current_state.mode << endl;

            bool TESTING = true;
            if(!current_state.mode.compare("OFFBOARD") || TESTING)
            {
                // ====================================
                // Start timing the search for flower state
                // start_search_time = time(nullptr);
                // gettimeofday(&tp, NULL);
                start_search_time = getTime();

                // Input Variables
                double r_max = 5;           // maximum radius of search in meters
                double v_max = 0.5;           // frequency of generatet setpoints in Hz -- must be larger than 2Hz
                double f_hz = 5;            // maximum velocity in m/s
                double search_height = 1.5;   // search height in meters

                // Publish the initialization msg
                drone_architecture::initializeSearch InitSearch;
                InitSearch.region_type = "SPIRAL";
                InitSearch.x_axis = r_max;
                InitSearch.y_axis = v_max;
                InitSearch.height = search_height;
                InitSearch.density = f_hz;
                search_pub.publish(InitSearch);
                cout << "=========== SENDING SEARCH INFO ============" << endl;

                // send to search algorithm state
                choice = cyclical_search;
                // ====================================
            }
            else
            {
                choice = searchForFlower;
            }
        }
        break;

        case cyclical_search:
        {
            double elapsed_time = getTime() - start_search_time;

            if(elapsed_time > 1/search_pattern_freq)
            {
                // Reset the timer
                // cout << "======= GO TO NEW WAYPOINT ========" << endl;
                // cout << "Target Freq: " << search_pattern_freq << "   Actual Freq: " << 1/elapsed_time << "   Latency: " << elapsed_time - 1/search_pattern_freq << endl;
                start_search_time = getTime();

                // Make the next waypoint service call
                drone_architecture::nextWaypoint next_waypt_srv;
                next_waypt_srv.request.go_to_pose = true;
                next_waypt_srv.request.next_pose = true;
                next_waypt_srv.request.publish = true;

                if(next_waypoint_client.call(next_waypt_srv))
                {
                    // cout << "Waypoint Recieved?: " << (int)next_waypt_srv.response.arrived << endl;
                }
                else
                {
                    cout << "ERROR NEXT WAYPOINT SERVICE FAILURE" << endl;
                }
            }
        }
        break;

        case lockOnFlower:
        {
            // int numberOfFlowers = flowerDataMessage.flowerArray.size();
            // desiredID = rand() % numberOfFlowers;
            // cout << "lockOnFlower Case" << endl;
            // choice = allignServo; //centerFlower;

            // Stop waypoints from being called
            drone_architecture::nextWaypoint next_waypt_srv;
            next_waypt_srv.request.go_to_pose = false;
            next_waypt_srv.request.next_pose = false;
            next_waypt_srv.request.publish = false;

            if(next_waypoint_client.call(next_waypt_srv))
            {
                // cout << "Waypoint Recieved?: " << (int)next_waypt_srv.response.arrived << endl;
            }
            else
            {
                cout << "ERROR NEXT WAYPOINT SERVICE FAILURE" << endl;
            }

            choice = centerFlower;

            // Enter center on flower and perform velocity control
        }
        break;

        case orient_north:
        {
            if(!current_state.mode.compare("OFFBOARD"))
            {
                cout << "=========== PUBLISHING NORTH ORIENTATION ===========" << endl;

                // Use the last setpoint given, and orient north
                lastSetpoint = currentPosition.pose;
                // lastSetpoint.orientation = northOrientation.orientation;

                drone_architecture::pixhawkInterface InterfaceMessage;
                InterfaceMessage.pose = lastSetpoint;
                InterfaceMessage.flight_command = "POSITION";
                for(int i=0; i < 10; i++)
                {
                    pixHawk_pub.publish(InterfaceMessage);
                }
                // Give it a few seconds to orient itself
                sleep(2);

                choice = centerFlower;
            }
            else
            {
                choice = orient_north;
            }
        }
        break;

        case centerFlower:
        {
            // int setpointX = 160;
            // int setpointY = 120;
            // float x_velocity = pid_velocity_x.setVelocity(setpointX, targetFlower.m_X);
            // float y_velocity = pid_velocity_y.setVelocity(setpointY, targetFlower.m_Y);
            float x_velocity = 1;
            float y_velocity = 0;

            drone_architecture::pixhawkInterface InterfaceMessage;
            geometry_msgs::Twist velocity_message;

            int theta = ((int)compassHeading - 90)%360;
            float x_vel_enu = x_velocity * cos(theta * (PI/180)) - y_velocity * sin(theta * (PI/180));
            float y_vel_enu = -1*(x_velocity * sin(theta * (PI/180)) + y_velocity * cos(theta * (PI/180)));
            float z_vel_enu = 0;

            float max_vel = 1.5;
            if(x_vel_enu > max_vel)
            {
                x_vel_enu = max_vel;
            }
            if(x_vel_enu < -max_vel)
            {
                x_vel_enu = -max_vel;
            }
            if(y_vel_enu > max_vel)
            {
                y_vel_enu = max_vel;
            }
            if(y_vel_enu < -max_vel)
            {
                y_vel_enu = -max_vel;
            }

            velocity_message.linear.x = x_vel_enu;
            velocity_message.linear.y = y_vel_enu;
            velocity_message.linear.z = z_vel_enu;
            cout << "Center Flower Velocities: " << velocity_message.linear.x << ", " << velocity_message.linear.y << endl;
            
            InterfaceMessage.twist = velocity_message;
            InterfaceMessage.flight_command = "VELOCITY";

            double elapsed_time = getTime() - start_search_time;

            if(elapsed_time > 1/search_pattern_freq)
            {
                pixHawk_pub.publish(InterfaceMessage);
                start_search_time = getTime();
            }

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
    ros::Publisher search_pub = n.advertise<drone_architecture::initializeSearch>("/search_initialize", 5);
    
    ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1, flowerCallback);
    ros::Subscriber Pixhawk_sub = n.subscribe("mavros/local_position/pose", 1, pixhawkCallback);
    ros::Subscriber Compass_sub = n.subscribe("mavros/global_position/compass_hdg", 1, compassCallback);
    ros::Subscriber home_sub = n.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, home_cb);
    ros::Subscriber state_sub = n.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);

    ros::ServiceClient next_waypoint_client = n.serviceClient<drone_architecture::nextWaypoint>("/next_waypoint");
    ros::ServiceClient update_home_client = n.serviceClient<mavros_msgs::CommandHome>("/mavros/cmd/set_home");

    ros::Rate loop_rate(20);
    ros::spinOnce();

    while (ros::ok()) {
        State_Machine(EndEffector_pub, pixHawk_pub, search_pub, next_waypoint_client, update_home_client);
        ros::spinOnce();
        loop_rate.sleep();
    }
}
