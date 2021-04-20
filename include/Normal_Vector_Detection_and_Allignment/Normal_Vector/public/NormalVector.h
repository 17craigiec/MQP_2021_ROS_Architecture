#include <iostream>
#include <vector>
#include <tuple>

#include "../public/BoxInfo.h"

using namespace std;


class absAngle
{
public:
    absAngle(float curent_theta);
    float getAbsAngle(float curent_theta);

private:
    double m_num_rotations = 0;
    float m_prev_theta = 0;
};

class NormalVector
{
private:
    float theta_planar = -1;
    float theta_elevated = -1;
    vector<tuple<float, double, double>> m_data_vector;     // <theta, mag_hw_ratio, is_inverted>

    float getDistanceRotated();

public:
    NormalVector();
    ~NormalVector();

    int recordRotation();
};