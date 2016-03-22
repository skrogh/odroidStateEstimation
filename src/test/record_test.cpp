#include <iostream>
#include <fstream>
#include "ImuDriver.h"

#include "LiveSLAMWrapper.h"

#include <boost/thread.hpp>
#include "util/settings.h"
#include "util/globalFuncs.h"
#include "SlamSystem.h"


#include "IOWrapper/OpenCV/OpenCVImageStreamThread.h"
#include "IOWrapper/OpenCV/OpenCVOutput3DWrapper.h"
#include "IOWrapper/ImageDisplay.h"

#include <unistd.h>
#include <sys/syscall.h>



void
slamThreadFunction(lsd_slam::InputImageStream* inputStream, lsd_slam::Output3DWrapper* outputWrapper)
{
	std::clog << "Live Slam thread started. PID: " << (int) syscall(SYS_gettid) << std::endl;
	lsd_slam::LiveSLAMWrapper slamNode(inputStream, outputWrapper);
	slamNode.Loop();
}

int
main( int argc, char** argv )
{
	std::clog << "Main thread started. PID: " << (int) syscall(SYS_gettid) << std::endl;
	lsd_slam::displayDepthMap = false;
	
	//open Logging file
	std::ofstream imuFile;
	imuFile.open("imuData.txt", std::ofstream::out);

	// Start IMU grabber (spawns it's own thread)
	std::cout << "Started" << std::endl;
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",100);
	
	// Start LSD SLAM (we spawn a tread of ir)
	lsd_slam::InputImageStream* inputStream = new lsd_slam::OpenCVImageStreamThread();
	inputStream->setCalibration("calib.txt");
	inputStream->run();
	lsd_slam::Output3DWrapper* outputWrapper = new lsd_slam::OpenCVOutput3DWrapper(inputStream->width(), inputStream->height());

	boost::thread slamThread(slamThreadFunction, inputStream, outputWrapper);

	// Main code goes here
	while(1)
	{
		ImuMeas_t imuData;
		imuDriver.imuBuffer.popBlocking(imuData);
		imuFile << imuData.gyro[0] << " " << imuData.gyro[1] << " " << imuData.gyro[2] << " "
		<< imuData.acc[0] << " " << imuData.acc[1] << " " << imuData.acc[2] << " "
		<< imuData.dist << " " << imuData.distValid << " "
		<< std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count()
		<< std::endl;
	}

	// Exit code (logging? cleanup) goes here
	slamThread.join(); // Join thread before exiting, or the lingering refs. to I/O wrappers will cause undefined behaviour
	imuFile.close();
	return 0;
}