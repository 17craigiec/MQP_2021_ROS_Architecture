#include <sys/time.h>
#include <cstddef>
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
        float setVelocity(int vel, int current_val);
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

float PID_Driver::setVelocity(int val_setpoint, int current_val)
{
    int dt = calcDt();

    double err = val_setpoint - current_val;

    double p_val = m_p*err;
    m_i_val += m_i*err*dt;
    double d_val = m_d*(err-m_prev_err)/dt;

    float setpoint = (float)p_val + (float)m_i_val + (float)d_val;

    // All values past here are just being updated in memory
    m_prev_val = current_val;
    m_prev_err = err;

    return setpoint;
}