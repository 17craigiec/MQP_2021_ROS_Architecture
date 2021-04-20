
runme: main.o RPi_BNO055.o BoxInfo.o CalculateVector.o
	g++ -lpigpio main.o RPi_BNO055.o BoxInfo.o CalculateVector.o -o runme

main.o: ../main.cpp
	g++ -lpigpio -c ../main.cpp

RPi_BNO055.o: ../../RPi_BNO055/RPi_BNO055.cpp ../../RPi_BNO055/RPi_BNO055.h
	g++ -lpigpio -c ../../RPi_BNO055/RPi_BNO055.cpp

BoxInfo.o: ../../BoxInfo/BoxInfo.cpp ../../BoxInfo/BoxInfo.h
	g++ -c ../../BoxInfo/BoxInfo.cpp

CalculateVector.o: ../../Normal_Vector/private/CalculateVector.cpp ../../Normal_Vector/public/CalculateVector.h
	g++ -c ../../Normal_Vector/private/CalculateVector.cpp

clean:
	rm *.o runme
