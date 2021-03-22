#include "../public/CalculateVector.h"

using namespace std;


absAngle::absAngle(float current_theta)
{
    m_num_rotations = 0;
    m_prev_theta = current_theta;
}

float absAngle::getAbsAngle(float current_theta)
{
    float pi = 3.14159;

    float abs_angle;
    float delta_theta = current_theta - m_prev_theta;
    float delta_threshold = pi;

    if(abs(delta_theta) > delta_threshold)
    {
        // The system has seen a rolleover of the 360-0 boundary
        if(delta_theta > 0)
        {
            m_num_rotations--;
        }
        if(delta_theta < 0)
        {
            m_num_rotations++;
        }
        else
        {
            cout << "How did I get here? getAbsAngle()" << endl;
        }
    }

    // Update the previous theta member
    m_prev_theta = current_theta;
    abs_angle = m_num_rotations*2*pi + current_theta;
    return abs_angle;
}

float getDistanceRotated(vector<tuple<float, double, double>> data_vector){
/**
 * This function finds the magnitude of the angular distance travelled
 * @param: a vector of tuples<theta, height, width> representing a collection of datapoints
 * @return: a float represeting the maximum delta theta
 */

    if(data_vector.size() == 0)
    {
        return 0;
    }

    float min_theta = 999;
    float max_theta = -999;

    for(int i = 0; i < data_vector.size(); i++)
    {
        if(get<0>(data_vector[i]) < min_theta)
        {
            min_theta = get<0>(data_vector[i]);
        }
        if(get<0>(data_vector[i]) > max_theta)
        {
            max_theta = get<0>(data_vector[i]);
        }
    }
    return max_theta - min_theta;
}

normal_vector calculateNormalVector(vector<tuple<float, double, double>> data_vector){
/**
 * This function will calculate the normal vector for the 3d flower based on a list of rectangular values
 * @param: a vector of tuples<theta, height, width> representing at least a 90 degree rotation
 * @return: normal_vector<X, Y, Z>
 */ 
    // This tuple will constantly compared to and will record the ratio of greatest magnitue
    // <magnitude, theta, state_of_inversion>
    tuple<float, double, bool> ratio_of_greatest_magnitude(0, 0, false);

    // data_point, current_ratio_mag, h, and w variables are purely used as dummy variables to keep operations organized
    tuple<float, double, double> data_point;
    double current_ratio_mag;
    float theta;
    double h;
    double w;

    // To measure ratio magnitudes we need to invert the ratio when < 1
    // Therefore, it is important to recorded when the maximum ratio recorded was an inversed ratio and thereore its final pose must be phase shifted.
    bool state_of_inversion = false;

    for(int i = 0; i < data_vector.size()-1; i++)
    {
        data_point = data_vector[i];

        theta = get<0>(data_point);
        h = get<1>(data_point);
        w = get<2>(data_point);

        // Calulate the ratio magnitue
        // NOTE: If the ratio is < 1 the magnitude will be calculated as its reciprocal - 1
        if(h/w > 1)
        {
            state_of_inversion = false;
            current_ratio_mag = h/w - 1;
        }
        else
        {
            state_of_inversion = true;
            current_ratio_mag = w/h - 1;
        }
        
        // Compare ratio magnitude to previously calculated value
        if(current_ratio_mag > get<0>(ratio_of_greatest_magnitude))
        {
            // assign new MAGNITUDE
            get<0>(ratio_of_greatest_magnitude) = current_ratio_mag;
            // assign new THETA DISPLACEMENT
            get<1>(ratio_of_greatest_magnitude) = theta;
            // assign new STATE OF INVERSION
            get<2>(ratio_of_greatest_magnitude) = state_of_inversion;
        }
    }

    normal_vector return_vector = {theta, 0, 0};

    return return_vector;
}









