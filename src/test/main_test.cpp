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
#include "MsfOutput3DWrapper.h"

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
	std::ofstream logFile;
	logFile.open("logData.txt", std::ofstream::out);
	
	// Initialize kalman filter
	ekf::EstimatorDelayHider estimator;
	estimator.SetState( Eigen::Vector3d(0, 0, 0), 	// p_i_w
			Eigen::Vector3d(0,0,0),				// v_i_w
			Eigen::Quaterniond(1,0,0,0),		// q_i_w
			Eigen::Vector3d(0,0,0),				// b_omega
			Eigen::Vector3d(0,0,0),				// b_a
			log(1),								// lambda
			Eigen::Vector3d(0.05, 0, -0.04),// p_c_i
			Eigen::Quaterniond(0.3549, 0.6116, -0.6116, 0.3549),		// q_c_i
			Eigen::Vector3d(0,0,0),				// p_w_v
			Eigen::Quaterniond(1,0,0,0));		// q_w_v
	estimator.SetCalibration(0.02*0.02,			// sq_sigma_omega
			0.05*0.05,								// sq_sigma_a
			0.001*0.001,							// sq_sigma_b_omega
			0.001*0.001,							// sq_sigma_a_omega
			1/400.0,							// Delta_t
			Eigen::Vector3d(0,0,9.82),			// g
			0,									// noise of scaling for new KFs
			true);								// measurements are absolute (in contrast to incremental)
	Eigen::Matrix<double,28,1> P;
	P << 0, 0, 0,								// p_i_w
		0.2, 0.2, 0.2,								// v_i_w
		0.3, 0.3, 0,							// q_i_w
		0.01, 0.01, 0.01,							// b_omega
		0.01, 0.01, 0.01,								// b_a
		log(1),									// lambda
		0.005, 0.005, 0.005,					// p_c_i
		0.05, 0.05, 0.05,						// q_c_i
		0, 0, 0,								// p_w_v
		0, 0, 0;								// q_w_v
	P = 1*P.cwiseProduct(P);
	estimator.SetCovarianceDiagonal(P);
	estimator.estimatorFull.UpdateKeyframe(); // update initial camera pose (a bit hacky)


	// Start LSD SLAM (we spawn a tread of ir)
	lsd_slam::InputImageStream* inputStream = new lsd_slam::OpenCVImageStreamThread();
	inputStream->setCalibration("calib.txt");
	inputStream->run();
	lsd_slam::MsfOutput3DWrapper* outputWrapper = new lsd_slam::MsfOutput3DWrapper(inputStream->width(), inputStream->height(), &estimator);

	boost::thread slamThread(slamThreadFunction, inputStream, outputWrapper);

	// Start IMU grabber (spawns it's own thread)
	std::cout << "Started" << std::endl;
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",100);

	estimator.Start();

	// Main code goes here
	while(1)
	{
		ImuMeas_t imuData;
		imuDriver.imuBuffer.popBlocking(imuData);

		estimator.ImuMeasurement(Eigen::Vector3d(imuData.gyro[0],imuData.gyro[1],imuData.gyro[2]),
			Eigen::Vector3d(imuData.acc[0],imuData.acc[1],imuData.acc[2]),
			imuData.dist, imuData.distValid,
			std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count()
			);
		static int i = 0;
		if (i++%100==0)
		{
		}

	}

	// Exit code (logging? cleanup) goes here
	slamThread.join(); // Join thread before exiting, or the lingering refs. to I/O wrappers will cause undefined behaviour
	logFile.close();
	return 0;
}
