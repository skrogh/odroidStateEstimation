#include <iostream>
#include "imuDriver/src/ImuDriver.h"

int
main(void)
{
	std::cout << "Started" << std::endl;
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",100);
	return 0;
}