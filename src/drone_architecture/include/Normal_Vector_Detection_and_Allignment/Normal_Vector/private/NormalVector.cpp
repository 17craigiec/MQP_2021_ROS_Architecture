#include "../../Servo_Control/RpiServo.h"
#include "../public/NormalVector.h"
#include "../public/UART.h"

#include "../../RPi_BNO055/RPi_Sensor.h"
#include "../../RPi_BNO055/RPi_BNO055.h"
#include "../../RPi_BNO055/utility/imumaths.h"

#include <pigpio.h>
#include <cmath>

#define BNO055_SAMPLERATE_DELAY_MS (100)


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

    // Check for outlandish false data
    if(abs(delta_theta) > 2.15*pi)
    {
        cout << "Error: False heading data collected" << endl;
        return m_prev_theta;
    }

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

NormalVector::NormalVector()
{
    if(gpioInitialise() <0)
	{
		std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
	}

    // // Construct the BNO object to act as a compass
    // Adafruit_BNO055 bno = Adafruit_BNO055();
    // // Associate with i2c
    // bno._HandleBNO=i2cOpen(bno._i2cChannel,BNO055_ADDRESS_A,0);
    // bno.begin();
    // // Sleep to allow the BNO to think
    // gpioSleep(PI_TIME_RELATIVE, 0, 2000);
}

NormalVector::~NormalVector()
{

}

float NormalVector::getDistanceRotated(){
/**
 * This function finds the magnitude of the angular distance travelled
 * @param: a vector of tuples<theta, height, width> representing a collection of datapoints
 * @return: a float represeting the maximum delta theta
 */

    if(m_data_vector.size() == 0)
    {
        return 0;
    }

    float min_theta = 999;
    float max_theta = -999;

    for(int i = 0; i < m_data_vector.size(); i++)
    {
        if(get<0>(m_data_vector[i]) < min_theta)
        {
            min_theta = get<0>(m_data_vector[i]);
        }
        if(get<0>(m_data_vector[i]) > max_theta)
        {
            max_theta = get<0>(m_data_vector[i]);
        }
    }
    return max_theta - min_theta;
}


int NormalVector::recordRotation()
{
    float pi = 3.14159;

    // Construct the BNO object to act as a compass
    Adafruit_BNO055 bno = Adafruit_BNO055();
    bno._HandleBNO=i2cOpen(bno._i2cChannel,BNO055_ADDRESS_A,0);
    bno.begin();
    // Sleep to allow the BNO to think
    gpioSleep(PI_TIME_RELATIVE, 0, 2000);
    // Find starting theta
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    float start_theta = euler.x()*2*pi/360;

    float theta_mag_observed = pi/2;
    float theta_rotated = 0;
    UART my_uart("/dev/ttyAMA0");
    BoxInfo bi;
    absAngle heading(start_theta);

    // This tuple will constantly compared to and will record the ratio of greatest magnitue
    // <theta, magnitude, state_of_inversion>
    tuple<float, double, bool> ratio_of_greatest_magnitude(0, 0, false);

    double current_ratio_mag;
    float theta;

    // To measure ratio magnitudes we need to invert the ratio when < 1
    // Therefore, it is important to recorded when the maximum ratio recorded was an inversed ratio and thereore its final pose must be phase shifted.
    bool state_of_inversion = false;

    while(theta_rotated < theta_mag_observed)
    {
        euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        theta = heading.getAbsAngle(euler.x()*2*pi/360);
        cout << "Theta: " << theta << endl;

        theta_rotated = getDistanceRotated();
        bi = readBoxInfo(my_uart.returnCurrentLine());

        // Calulate the ratio magnitue
        // NOTE: If the ratio is < 1 the magnitude will be calculated as its reciprocal - 1
        if(bi.height/bi.width < 1)
        {
            state_of_inversion = false;
            current_ratio_mag = 1 - bi.height/bi.width;
        }
        else
        {
            state_of_inversion = true;
            current_ratio_mag = 1 - bi.width/bi.height;
        }
        
        // Compare ratio magnitude to previously calculated value
        if(current_ratio_mag > get<1>(ratio_of_greatest_magnitude))
        {
            // assign new THETA
            get<0>(ratio_of_greatest_magnitude) = theta;
            // assign new MAGNITUDE RATIO
            get<1>(ratio_of_greatest_magnitude) = current_ratio_mag;
            // assign new STATE OF INVERSION
            get<2>(ratio_of_greatest_magnitude) = state_of_inversion;
        }

        m_data_vector.push_back({theta, current_ratio_mag, state_of_inversion});
    }

    state_of_inversion = get<2>(ratio_of_greatest_magnitude);
    double planar_vector_theta = get<0>(ratio_of_greatest_magnitude);
    double elevated_vector_theta = acos(get<1>(ratio_of_greatest_magnitude));

    // If inverted measurement, offset by 90deg or pi/2 rad
    if(!state_of_inversion)
    {
        planar_vector_theta += pi/2;
    }
    
    cout << "System has recorded data over a range of " << theta_rotated << " radians..." << endl;
    cout << "Planar components of normal vector detected to be: " << planar_vector_theta << endl;
    cout << "Maximum ratio: " << get<1>(ratio_of_greatest_magnitude) << endl;
    cout << "Elevtated theta of from the horizontal plane: " << elevated_vector_theta << endl;

    // Get this to later run outside this function
    RpiServo sv(4);
    cout << "test" << endl;
    // sv.setServo((elevated_vector_theta)*2000/pi);
    sv.setServo(500);
    cout << "test2.0" << endl;

    my_uart.join();
    return 0;
}