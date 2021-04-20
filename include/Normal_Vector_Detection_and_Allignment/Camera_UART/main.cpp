// THIS FILE CONTAINS EXAMPLE OPERATION OF UART COMMUNICATION :: Open_MV --> Pi0w
#include <vector>
#include <tuple>
#include <stdio.h>
#include <sys/ioctl.h>
#include "../BoxInfo/BoxInfo.h"

//Linux headers
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>

// Compass related includes
#include <pigpio.h>
#include "../RPi_BNO055/RPi_Sensor.h"
#include "../RPi_BNO055/RPi_BNO055.h"
#include "../RPi_BNO055/utility/imumaths.h"

#include "../Normal_Vector/public/CalculateVector.h"

#define BNO055_SAMPLERATE_DELAY_MS (100)

using namespace std;



int main(int argc, char const *argv[])
{
    (void)argc;
    (void)argv;

    // Example code begins here
    // ====================================================================

    // Construct the BNO object to act as a compass
    Adafruit_BNO055 bno = Adafruit_BNO055();
    // Init bno
    if (gpioInitialise() <0)
	{
		std::cout <<"Initialisation error of the GPIO \n Closing program..."<< std::endl;
		return -1;
	}
    // Associate with i2c
    bno._HandleBNO=i2cOpen(bno._i2cChannel,BNO055_ADDRESS_A,0);
    bno.begin();
    // Sleep to allow the BNO to think
    gpioSleep(PI_TIME_RELATIVE, 0, 1000);


    // running_bool
    bool test_running = true;
    double pi = 3.14159;

    // Find starting theta
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    double start_theta = euler.x()*2*pi/360;

    // Comapss Initialization END
    // ==============================================


    // Datatype: vector<tuple<theta, height, width>>
    vector<tuple<float, double, double>> collection_of_datapoints;
    double flower_normal_theta = 0;
    normal_vector flower_norm;
    BoxInfo bi;

    // Values used for mapping theta
    // double prev_theta = start_theta;
    // double current_theta = start_theta;
    // double calculated_theta;
    // int num_jumps = 0;
    float current_theta = start_theta;
    absAngle abs_angle_calculator(start_theta);

    while(test_running)
    {
        // collect compass heading
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        // This is a error check to assure the values are within the arruate range
        if(abs(euler.x()*2*pi/360) < 2.1*pi)
        {
            current_theta = euler.x()*2*pi/360;
        }
        
        // collect the heading and map it to an -inf <--> inf scale
        // double mapped_theta = mapTheta(current_theta, prev_theta, num_jumps);
        float mapped_theta = abs_angle_calculator.getAbsAngle(current_theta);
        cout << "Heading: " << mapped_theta << endl;
        
        // bi = readBoxInfo();
        ifstream myfile ("/dev/ttyAMA0");
        string line;

        getline(myfile, line);
        cout << line << endl;

        // bi.height = 35;
        // bi.width = 40;
        // bi.x = 120;
        // bi.y = 55;

        // if(current_theta != 0)
        // {
        //     if(getDistanceRotated(collection_of_datapoints) < pi/2 && isBoxValid(bi))
        //     {
        //         collection_of_datapoints.push_back(tuple<float, double, double>(mapped_theta, bi.height, bi.width));
        //         cout << "This is how far youve travelled: " << getDistanceRotated(collection_of_datapoints) << endl;
        //         flower_norm = calculateNormalVector(collection_of_datapoints);

        //         // float flower_normal_theta = atan2(flower_norm.i,flower_norm.j);
        //         float flower_normal_theta = flower_norm.i;
        //     }

        //     // send a UART com back to the camera
        //     // double calculated_theta = -1*flower_normal_theta + current_theta - pi/2;
        //     // cout << "Cacluated Theta Value: " << calculated_theta << endl;
        //     // myfile << "*_" << calculated_theta << "*_" << endl;
        //     // myfile << "*_" << endl;

        // }

        // if(abs(current_theta - flower_normal_theta) < 0.1)
        // {
        //     cout << "You're aligned with the normal vector!    Current Theta: " << current_theta << "    Flower Normal Theta: " << flower_normal_theta << endl;
        // }
    }
    
    
    // Exit the test script
    return 0;
}
