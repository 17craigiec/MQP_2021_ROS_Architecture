#include <tuple>
#include <vector>
#include <math.h>
#include <cstdlib>
#include <iostream>

using namespace std;

struct normal_vector
{
    double i;
    double j;
    double k;
};

class absAngle
{
    public:
        absAngle(float curent_theta);
        float getAbsAngle(float curent_theta);

    private:
        double m_num_rotations = 0;
        float m_prev_theta = 0;
};

float getDistanceRotated(vector<tuple<float, double, double>> data_vector);
normal_vector calculateNormalVector(vector<tuple<float, double, double>> data_vector);