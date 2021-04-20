#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/HomePosition.h>
#include "drone_architecture/pixhawkInterface.h"
#include "drone_architecture/initializeSearch.h"
#include "drone_architecture/searchStatus.h"
#include "drone_architecture/nextWaypoint.h"
#include <iostream>
#include <string.h>
#include <math.h>
#include <vector>


drone_architecture::searchStatus current_status;
std::vector<float> current_waypoint(3);

void generate_waypoints(std::vector<std::vector<float>> * waypoints, int size, const std::string& type, float x_axis, float y_axis, float height, float density){

    std::vector<std::vector<float>> points(size);
    ROS_INFO("Size: %li", points.size());

    std::vector<float> temp(3);
    for (int i = 0; i < size; i++){
        points[i] = temp;
    }

    if (type.compare("RECTANGLE") == 0){
        ROS_INFO("Rectangle!");
        current_status.region_type = "RECTANGLE";
        bool decrease{false};
        int count{0};
        for (float i = 0; i <= y_axis; i += density) {
            if (decrease){
                for (float j = x_axis; j >= 0; j -= density){
                    points[count][0] = j;
                    points[count][1] = i;
                    points[count][2] = height;
                    count++;
                }
            } else {
                for (float j = 0; j <= x_axis; j += density) {
                    points[count][0] = j;
                    points[count][1] = i;
                    points[count][2] = height;
                    count++;
                }
            }
            decrease = !decrease;
        }
    }
    else if (type.compare("ELLIPSE") == 0){
        ROS_INFO("Ellipse!");
    }
    else if (type.compare("SPIRAL") == 0){
        ROS_INFO("Spiral!");
        current_status.region_type = "SPIRAL";

        std::cout << "========= BEGIN SPIRAL REGION ==========" << std::endl;

        double v_max = y_axis; // maximum velocity in m/s
        double f_hz = density; // setpoint frequency in 1/s

        double a = 1; // distance between spirals in meters
        double c = 1.5855; // a magic constant

        // Create the list of setpoints
        for(int i=0; i < size; i++){
            double t = (double)i/f_hz;
            // std::cout << t << std::endl;

            double theta = c*sqrt((v_max*t*2*M_PI)/a);
            double x_pos = ((a*theta)/(2*M_PI))*cos(theta);
            double y_pos = ((a*theta)/(2*M_PI))*sin(theta);
            double z_pos = height; // flight height 

            points[i][0] = x_pos;
            points[i][1] = y_pos;
            points[i][2] = z_pos;
            std::cout << x_pos << ", " << y_pos << std::endl;
        }

        std::cout << "========= END SPIRAL REGION ==========" << std::endl;
    }

    *waypoints = points;
} 

bool init_waypoints = false;
drone_architecture::initializeSearch init_msg;
int num_points;
void init_cb(const drone_architecture::initializeSearch::ConstPtr& msg){
    init_msg = *msg;
    init_waypoints = true;
    
    if (msg->region_type.compare("RECTANGLE") == 0){
        num_points = int((msg->x_axis + msg->density)/msg->density * (msg->y_axis + msg->density)/msg->density);
    }
    // else if (type.compare("ELLIPSE") == 0){

    // }
    else if (msg->region_type.compare("SPIRAL") == 0){
        double r_max = msg->x_axis; // maximum radius of search in meters
        double v_max = msg->y_axis; // frequency of generatet setpoints in Hz -- must be larger than 2Hz
        double f_hz = msg->density; // maximum velocity in m/s

        
        double a = 2; // distance between spirals in meters
        double c = 1.5855; // a magic constant

        double max_theta = r_max*2*M_PI/a; // maximum theta reached in radians
        double search_time = pow((max_theta/c),2)*(a/(v_max*2*M_PI)); //time elapsed to search a given radian theta
        num_points = floor(search_time*f_hz); // number of setpoint generated in a given time to reach a certain angle in radians
        std::cout << "========= SPIRAL REGION INITIALIZED ==========" << std::endl;
    }
}

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

mavros_msgs::HomePosition home_pose;
void home_cb(const mavros_msgs::HomePosition::ConstPtr& msg){
    home_pose = *msg;
}

bool update_waypoint_flag{false};
bool publish_target{false};
float ep_x{0.1};
float ep_y{0.1};
float ep_z{0.1};
bool next_waypoint(drone_architecture::nextWaypoint::Request  &req,
                   drone_architecture::nextWaypoint::Response &res)
{
    
    bool retval = false;
    if (req.next_pose){
        update_waypoint_flag = true;
        res.arrived = false;
        retval = true;
    } else {
        res.arrived = false;
        retval = true;
    }

    if(req.go_to_pose){
        if (abs(current_waypoint[0] - current_pose.pose.position.x) < ep_x && 
            abs(current_waypoint[1] - current_pose.pose.position.y) < ep_y &&
            abs(current_waypoint[2] - current_pose.pose.position.z) < ep_z){
            res.arrived = true;
            retval = true;
        } else {
            res.arrived = false;
            retval = true;
        }
    }

    publish_target = req.publish;
    current_status.publishing = req.publish;

    return retval;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "SearchRegion");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber home_sub = nh.subscribe<mavros_msgs::HomePosition>
            ("mavros/home_position/home", 10, home_cb);
    ros::Subscriber init_sub = nh.subscribe<drone_architecture::initializeSearch>
            ("search_initialize", 10, init_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);

    ros::Publisher interface_pub = nh.advertise<drone_architecture::pixhawkInterface>
        ("command", 1);
    ros::Publisher status_pub = nh.advertise<drone_architecture::searchStatus>
        ("search_status", 10);
    current_status.region_type = "UNDEFINED";
    current_status.status = "NO_REGION";

    ros::ServiceServer waypoint_service = nh.advertiseService("next_waypoint", next_waypoint);

    // //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    while(ros::ok() && !init_waypoints){
        ros::spinOnce();
        rate.sleep();
    }

    std::vector<std::vector<float>> waypoints(num_points);

    generate_waypoints(&waypoints, num_points, init_msg.region_type, init_msg.x_axis, init_msg.y_axis, init_msg.height, init_msg.density);
    
    int waypoint_index{0};
    current_waypoint = waypoints[waypoint_index];

    current_status.total_waypoints = num_points;
    current_status.current_waypoint = 0;

    // wait for FCU connection
    while(ros::ok() && !current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Connection Established");

    current_status.status = "SEARCHING";

    drone_architecture::pixhawkInterface goto_waypoint;
    goto_waypoint.pose.position.x = home_pose.position.x + current_waypoint[0];
    goto_waypoint.pose.position.y = home_pose.position.y + current_waypoint[1];
    goto_waypoint.pose.position.z = home_pose.position.z + current_waypoint[2];
    goto_waypoint.pose.orientation = home_pose.orientation;
    goto_waypoint.flight_command = "POSITION";

    while(ros::ok()){

        if (update_waypoint_flag) {

            // Update to next waypoint if there is another one
            if (waypoint_index < num_points - 1){
                waypoint_index++;
            } else {
                current_status.status = "DONE";
            }

            // Set the new current waypoint
            current_waypoint = waypoints[waypoint_index];

            goto_waypoint.pose.position.x = home_pose.position.x + current_waypoint[0];
            goto_waypoint.pose.position.y = home_pose.position.y + current_waypoint[1];
            goto_waypoint.pose.position.z = home_pose.position.z + current_waypoint[2];

            current_status.current_waypoint = waypoint_index + 1;

            update_waypoint_flag = false;
        }

        if (publish_target){
            interface_pub.publish(goto_waypoint);
        }

        status_pub.publish(current_status);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}