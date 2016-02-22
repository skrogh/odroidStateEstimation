# odroidStateEstimation
Combining visual odometry provided by LSD-SLAM with IMU measurements through fusion with an ekf

## Algorithms used:
LSD-SLAM: http://vision.in.tum.de/research/vslam/lsdslam (https://github.com/tum-vision/lsd_slam)
EKF: http://e-collection.library.ethz.ch/eserv/eth:5889/eth-5889-02.pdf (https://github.com/ethz-asl/ethzasl_msf)

## Goal
Provide state estimation (primarilty velocity) for my custom quadrotor based on an oDroid Linux computer and custom flightcontroller hardware.
(previously with featurebased estimation: https://www.youtube.com/watch?v=EyRYwbe2Al8)
