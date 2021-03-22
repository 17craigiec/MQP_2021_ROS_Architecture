# g++ -Wall -lpthread -lpigpio -lrt test_normal_vector.cpp ./private/BoxInfo.cpp ./private/NormalVector.cpp ./private/UART.cpp ../RPi_BNO055/RPi_BNO055.cpp -o test_normal_vector

g++ -Wall -lpthread -lpigpio -lrt test_normal_vector.cpp ./private/BoxInfo.cpp ./private/NormalVector.cpp ./private/UART.cpp ../Servo_Control/RpiServo.cpp ../RPi_BNO055/RPi_BNO055.cpp -o test_normal_vector

echo BUILD COMPLETE
