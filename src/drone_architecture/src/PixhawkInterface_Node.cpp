#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include "drone_architecture/pixhawkInterface.h"
#include <string.h>



mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

drone_architecture::pixhawkInterface incoming_msg;
void command_cb(const drone_architecture::pixhawkInterface::ConstPtr& msg){
    incoming_msg = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "PixhawkInterface");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber command_sub = nh.subscribe<drone_architecture::pixhawkInterface>
            ("/command", 10, command_cb);


    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::Publisher local_vel_pub = nh.advertise<geometry_msgs::Twist>
            ("mavros/setpoint_velocity/cmd_vel_unstamped", 10);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connection Established");


    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    geometry_msgs::PoseStamped target_pose;
    geometry_msgs::Twist target_twist;


    while(ros::ok()){
        std::string flight_mode = incoming_msg.flight_command;

        if(flight_mode.compare("ARM") == 0){
            if(arming_client.call(arm_cmd) && arm_cmd.response.success){
                ROS_INFO("Vehicle armed");
            }
        }

        if(flight_mode.compare("POSITION") == 0){
            target_pose.pose = incoming_msg.pose;
            local_pos_pub.publish(target_pose);
        }

        if(flight_mode.compare("VELOCITY") == 0){
            target_twist = incoming_msg.twist;
            local_vel_pub.publish(target_twist);
        }

        if(flight_mode.compare("OFFBOARD") == 0){

            if(set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}