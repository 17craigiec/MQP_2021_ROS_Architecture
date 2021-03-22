#include "ros/ros.h"
#include "std_msgs/String.h"
#include "drone_architecture/normalVectorTrigger.h"
#include "drone_architecture/flower.h"
#include "drone_architecture/normalVectorStatus.h"


class NormalVectorCalculator_Node
{
private:
  /* data */
public:
  NormalVectorCalculator_Node(/* args */);
  ~NormalVectorCalculator_Node();
};

NormalVectorCalculator_Node::NormalVectorCalculator_Node(/* args */)
{
}

NormalVectorCalculator_Node::~NormalVectorCalculator_Node()
{
}



/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void normalVectorCallback(const drone_architecture::normalVectorTrigger& msg)
{
    std::cout<<"Calculating Normal Vector"<<std::endl;
    std::cout<<msg.m_flower<<std::endl;

    // Requirements
    // constant stream of flower data
    // constant stream of angular heading
    
    // Stored in a global class
    // flower #
    // a list of <heading, h:w magnitude, bool_ratio_inversion>
}

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
  ros::init(argc, argv, "NormalVectorCalculator_Node");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("calculateNormalVector", 1000, normalVectorCallback);
  ros::Publisher normalVectorStatus_pub = n.advertise<drone_architecture::normalVectorStatus>("normalVectorStatus", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();

  return 0;
}