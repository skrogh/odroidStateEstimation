/**
* This file is part of LSD-SLAM.
*
* Copyright 2013 Jakob Engel <engelj at in dot tum dot de> (Technical University of Munich)
* For more information see <http://vision.in.tum.de/lsdslam> 
*
* LSD-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* LSD-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with LSD-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MsfOutput3DWrapper.h"
#include "util/settings.h"
#include "util/SophusUtil.h"
#include "util/globalFuncs.h"
#include "EstimatorDelayHider.h"

#include "DataStructures/Frame.h"
#include "GlobalMapping/KeyFrameGraph.h"
#include "sophus/sim3.hpp"
#include "GlobalMapping/g2oTypeSim3Sophus.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <boost/thread.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <stdio.h>



namespace lsd_slam
{


MsfOutput3DWrapper::MsfOutput3DWrapper(int width, int height, ekf::EstimatorDelayHider *output)
{
	this->width = width;
	this->height = height;
	this->output = output;
	outputVideo.open("KeyFrames.mpeg", cv::VideoWriter::fourcc('P','I','M','1'),30,cv::Size(width,height));
	outputVideoVar.open("KeyFramesVar.mpeg", cv::VideoWriter::fourcc('P','I','M','1'),30,cv::Size(width,height));
	outputVideoCombi.open("KeyFramesCombi.mpeg", cv::VideoWriter::fourcc('P','I','M','1'),30,cv::Size(width,height));
}

MsfOutput3DWrapper::~MsfOutput3DWrapper()
{
	
}


void MsfOutput3DWrapper::publishKeyframe(Frame* f)
{
	// Debug print, but pass no data to kalmanfilter
	cv::Mat img;
	cv::Mat imgVar;
	{
		boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
		img = getDepthRainbowPlot(f);
		imgVar = getVarRedGreenPlot( f->idepthVar(), f->image(), f->width(), f->height());
	}	
	outputVideo << img;
	outputVideoVar << imgVar;
	outputVideoCombi << img;
	outputVideoCombi << imgVar;
}

void MsfOutput3DWrapper::publishTrackedFrame(Frame* f)
{	
	boost::shared_lock<boost::shared_mutex> lock = f->getActiveLock();
	
	bool isNewKeyframe = false;
	if (f->pose->trackingParent!=nullptr)
	{
		Frame* parent = f->pose->trackingParent->frame;
		boost::shared_lock<boost::shared_mutex> parentLock = parent->getActiveLock();

		Eigen::Vector3d keyframeToWorld = parent->pose->getCamToWorld().translation();
		if (keyframeToWorldPrevious != keyframeToWorld)
			isNewKeyframe = true;
		keyframeToWorldPrevious = keyframeToWorld;
	}

	Eigen::Matrix<double,6,6> R = Eigen::Matrix<double,6,6>::Zero();
	R.diagonal()[0] = 0.01*0.01;
	R.diagonal()[1] = 0.01*0.01;
	R.diagonal()[2] = 0.01*0.01;
	R.diagonal()[3] = 0.1*0.1;
	R.diagonal()[4] = 0.1*0.1;
	R.diagonal()[5] = 0.05*0.05;

	Sim3 camToWorld = f->getScaledCamToWorld();
	output->CameraMeasurement(
		camToWorld.translation(),
		camToWorld.quaternion().normalized(),
		R,
		isNewKeyframe,
		f->timeStampNs());
}


void MsfOutput3DWrapper::publishKeyframeGraph(KeyFrameGraph* graph)
{
	// unimplemented
}

void MsfOutput3DWrapper::publishTrajectory(std::vector<Eigen::Matrix<float, 3, 1>> trajectory, std::string identifier)
{
	// unimplemented ... do i need it?
}

void MsfOutput3DWrapper::publishTrajectoryIncrement(Eigen::Matrix<float, 3, 1> pt, std::string identifier)
{
	// unimplemented ... do i need it?
}

void MsfOutput3DWrapper::publishDebugInfo(Eigen::Matrix<float, 20, 1> data)
{
	// std_msgs::Float32MultiArray msg;
	// for(int i=0;i<20;i++)
	// 	msg.data.push_back((float)(data[i]));

	// debugInfo_publisher.publish(msg);
}

}
