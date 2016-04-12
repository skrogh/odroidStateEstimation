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
#include <cmath>

#include <pthread.h>
#include <sched.h>

int setCpuAffinity( int num, const int* cores )
{
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	for( int i=0; i<num; i++ )
	{
		int num_cores = sysconf(_SC_NPROCESSORS_ONLN);
		if (cores[i] < 0 || cores[i] >= num_cores)
			return EINVAL;

		CPU_SET(cores[i], &cpuset);
	}

	pthread_t current_thread = pthread_self();    
	return pthread_setaffinity_np(current_thread, sizeof(cpu_set_t), &cpuset);
}

int setThreadPrioirtyMax( void )
{
	pthread_t thId = pthread_self();
    pthread_attr_t thAttr;
    int policy = 0;
    int max_prio_for_policy = 0;

    pthread_attr_init(&thAttr);
    pthread_attr_getschedpolicy(&thAttr, &policy);
    max_prio_for_policy = sched_get_priority_max(policy);


    pthread_setschedprio(thId, max_prio_for_policy);
    pthread_attr_destroy(&thAttr);

    return 0;
}

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

	{ // Ensure that LSD-slam does not use CPU0
		int coresActive[] = {1,2,3};
		setCpuAffinity(3, coresActive);
	}

	std::clog << "Main thread started. PID: " << (int) syscall(SYS_gettid) << std::endl;
	lsd_slam::displayDepthMap = false;
	
	//open Logging file
	std::ofstream logFile;
	logFile.open("logData.txt", std::ofstream::out);
	
	// Initialize kalman filter
	ekf::EstimatorDelayHider estimator;
	estimator.SetState( Eigen::Vector3d(0, 0, 0.2), 	// p_i_w
			Eigen::Vector3d(0,0,0),				// v_i_w
			Eigen::Quaterniond(1,0,0,0),		// q_i_w
			Eigen::Vector3d(0,0,0),				// b_omega
			Eigen::Vector3d(0,0,0),				// b_a
			log(1),								// lambda
			Eigen::Vector3d(0.05, 0, -0.10),// p_c_i
			Eigen::Quaterniond(1, 1, -1, 1),		// q_c_i
			Eigen::Vector3d(0,0,0),				// p_w_v
			Eigen::Quaterniond(1,0,0,0));		// q_w_v
	estimator.SetCalibration(0.02*0.02,			// sq_sigma_omega
			0.05*0.05,								// sq_sigma_a
			0.001*0.001,							// sq_sigma_b_omega
			0.001*0.001,							// sq_sigma_a_omega
			1/400.0,							// Delta_t
			Eigen::Vector3d(0,0,9.82),			// g
			0.0001,									// noise of scaling for new KFs
			true);								// measurements are absolute (in contrast to incremental)
	Eigen::Matrix<double,28,1> P;
	P << 0, 0, 0.1,								// p_i_w
		0.2, 0.2, 0.2,								// v_i_w
		0.3, 0.3, 0,							// q_i_w
		0.01, 0.01, 0.01,							// b_omega
		0.01, 0.01, 0.01,								// b_a
		log(2),									// lambda
		0.005, 0.005, 0.005,					// p_c_i
		0.05, 0.05, 0.05,						// q_c_i
		0, 0, 0,								// p_w_v
		0, 0, 0;								// q_w_v
	P = 1*P.cwiseProduct(P);
	estimator.SetCovarianceDiagonal(P);
	estimator.estimatorFull.UpdateKeyframe(); // update initial camera pose (a bit hacky)


	// Set LSD slam parameters
	cv::Mat occlusion = cv::imread("occlusion.png", CV_LOAD_IMAGE_GRAYSCALE );
	lsd_slam::occludedImage.BuildFromImage( occlusion );

	// Start LSD SLAM (we spawn a tread of it)
	lsd_slam::InputImageStream* inputStream = new lsd_slam::OpenCVImageStreamThread();
	inputStream->setCalibration("calib.txt");
	inputStream->run();
	lsd_slam::MsfOutput3DWrapper* outputWrapper = new lsd_slam::MsfOutput3DWrapper(inputStream->width(), inputStream->height(), &estimator);

	boost::thread slamThread(slamThreadFunction, inputStream, outputWrapper);

	// Start IMU grabber (spawns it's own thread)
	std::cout << "Started" << std::endl;
	ImuDriver imuDriver("/dev/spidev1.0","/sys/class/gpio/gpio199/value",400);

	estimator.Start();

	{	// Let main thread use any core (if running on system with more/less cores adjust)
		// Set max priority
		int coresActive[] = {0,1,2,3};
		setCpuAffinity(3, coresActive);
		setThreadPrioirtyMax();
	}
	// Main code goes here
	unsigned long long int imuTimePrevious = 0;
	unsigned int loopCounter = 0;
	double ix = 0;
	double iy = 0;
	while(1)
	{
		ImuMeas_t imuData;
		imuDriver.imuBuffer.popBlocking(imuData);
		unsigned long long int imuTime = std::chrono::duration_cast<std::chrono::nanoseconds>(imuData.timeStamp.time_since_epoch()).count();
		
		if ( imuTime - imuTimePrevious > ( 2500000 + 1000000 ) )
			std::cerr << "MAIN THEAD WARNING! IMU masurement is late (probably missed one). Dt: "
			<< imuTime - imuTimePrevious <<
			" Missed: " <<  ((imuTime - imuTimePrevious)/2500000.0f - 1.0f) << std::endl;
		imuTimePrevious = imuTime;

		Eigen::Vector3d a_m = Eigen::Vector3d(imuData.acc[0],imuData.acc[1],imuData.acc[2]);
		Eigen::Vector3d omega_m = Eigen::Vector3d(imuData.gyro[0],imuData.gyro[1],imuData.gyro[2]);

		estimator.ImuMeasurement(omega_m,
			a_m,
			imuData.dist, imuData.distValid,
			imuTime
			);
		static int i = 0;
		if (i++%10==0)
		{
			logFile << estimator.estimatorPredictorCurrent.p_i_w.transpose() << " "
				<< estimator.estimatorPredictorCurrent.v_i_w.transpose() << " "
				<< estimator.estimatorPredictorCurrent.q_i_w.q.coeffs().transpose() << " "
				<< (estimator.estimatorPredictorCurrent.q_i_w.toQuat().toRotationMatrix()*estimator.estimatorPredictorCurrent.v_i_w).transpose() << std::endl;
		}
		// Controller goes here
		// Calculate local speed
		
		Eigen::Vector3d v_i_hat = (estimator.estimatorPredictorCurrent.q_i_w.toQuat().toRotationMatrix()*estimator.estimatorPredictorCurrent.v_i_w).transpose();
		Eigen::Vector3d a_i_hat = a_m - estimator.estimatorPredictorCurrent.b_a - estimator.estimatorPredictorCurrent.q_i_w.toQuat().toRotationMatrix()*estimator.estimatorPredictorCurrent.g;
		Eigen::Vector3d omega_hat = omega_m - estimator.estimatorPredictorCurrent.b_omega;

		Eigen::Vector3d v_sp( imuData.steerX/2.0, imuData.steerY/2.0,0 );
		Eigen::Vector3d v_err = v_i_hat - v_sp;

		ix -= 10*v_err(0)/400.0f;
		iy -= 10*v_err(1)/400.0f;
		ix = std::min(std::max(ix,-0.5),0.5);
		iy = std::min(std::max(iy,-0.5),0.5);

		imuDriver.SetOutput( -10*v_err(0)-0.2*a_i_hat(0)+ix, -5*v_err(1)-0.2*a_i_hat(1)+iy, 0, 0.25+0.05*sin(M_PI_2*loopCounter/400.0f) );
		loopCounter++;
	}

	// Exit code (logging? cleanup) goes here
	slamThread.join(); // Join thread before exiting, or the lingering refs. to I/O wrappers will cause undefined behaviour
	logFile.close();
	return 0;
}
