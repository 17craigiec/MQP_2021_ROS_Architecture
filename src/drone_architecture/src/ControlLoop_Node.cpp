#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

//===========================================================
//======================== PID ==============================

class PID_Driver
{
private:
    double m_p = -1;
    double m_i = -1;
    double m_d = -1;

    double m_i_val = 0;
    double m_prev_val = 0;
    double m_prev_err = 0;
    double m_prev_time = 0;
public:
    PID_Driver(double p, double i, double d);
    ~PID_Driver();

    // vel is a double bounded from 0 to inf
    void setVelocity(double vel);
    double timestamp();
    int calcDt();
};

PID_Driver::PID_Driver(double p, double i, double d)
{
    m_p = p;
    m_i = i;
    m_d = d;
}

PID_Driver::~PID_Driver()
{
  
}

double PID_Driver::timestamp()
{
    struct timeval tp;
    gettimeofday(&tp, NULL);
    return tp.tv_sec * 1000 + tp.tv_usec / 1000;
}

int PID_Driver::calcDt()
{
    int current_time = timestamp();
    int dt = current_time - m_prev_time;
    m_prev_time = current_time;
    return dt;
}

double PID_Driver::setVelocity(double val_setpoint, int current_val)
{
    int dt = calcDt();

    double err = val_setpoint - current_val;

    double p_val = m_p*err;
    m_i_val += m_i*err*dt;
    double d_val = m_d*(err-m_prev_err)/dt;

    double setpoint = p_val + m_i_val + d_val;

    // All values past here are just being updated in memory
    m_prev_val = current_vel;
    m_prev_err = err;

    return setpoint
}

//===========================================================


using namespace std;
void flowerCallback(const drone_architecture::flowerData& msg)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ControlLoop_Node");
    ros::NodeHandle n;

    //ros::Publisher CalculateNormalVector_pub = n.advertise<drone_architecture::normalVectorTrigger>("calculateNormalVector", 1000);
    
    //ros::Subscriber Flower_sub = n.subscribe("FlowerData", 1000, flowerCallback);

    ros::Rate loop_rate(10);
    ros::spinOnce();

    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }
}
