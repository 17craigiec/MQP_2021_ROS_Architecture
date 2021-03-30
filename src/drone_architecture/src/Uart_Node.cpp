#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/flowerData.h"
#include <sstream>
#include <iostream>
#include <fstream>
#include <stdio.h> 
#include <string.h> 

using namespace std; 

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "Uart_Node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. 
   */
  ros::Publisher FlowerData_pub = n.advertise<drone_architecture::flowerData>("FlowerData", 1000);

  ros::Rate loop_rate(10);

  ifstream myfile ("/dev/ttyAMA0");
  string line;
  while (ros::ok())
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    drone_architecture::flowerData msg;
    getline(myfile, line);
    std::string delimiter = ")";
    size_t pos = 0;
    string token;
    while ((pos = line.find(delimiter)) != string::npos) {
        drone_architecture::flower flower;
        bool goodMessage = true;
        token = line.substr(0, pos);
        token.erase(remove(token.begin(), token.end(), '('), token.end()); 
        string flowerDelimiter = ",";
        size_t flowerPos = 0;
        string flowerToken;
        int index = 0;
        while ((flowerPos = token.find(flowerDelimiter)) != std::string::npos) {
            try{
              flowerToken = token.substr(0, flowerPos);
              string::size_type sz;   // alias of size_t
              if(index == 0){
                flower.m_ID = stoi (flowerToken,&sz);
              }
              else if(index ==1){
                flower.m_X = stoi (flowerToken,&sz);
              }
              else{
                goodMessage = false;
              }
              index++;
              token.erase(0, flowerPos + flowerDelimiter.length());
            }
            catch(int e)
            {
              goodMessage = false;
              cout<<"Error Occured when converting string to int in UART_NODE"<<endl;
            }  
        }
        if(goodMessage){
          string::size_type sz;   // alias of size_t
          flower.m_Y = stoi (flowerToken,&sz);
          msg.flowerArray.push_back(flower);
        }
        line.erase(0, pos + delimiter.length());
    }
    FlowerData_pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

