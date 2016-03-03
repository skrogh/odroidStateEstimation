cd external/imuDriver
make -j3
cd ../../

cd external/msf_ekf
make -j3
cd ../../

cd external/lsd_slam/lsd_slam_core
make -j3
cd ../../../

cd build
make -j3
cd ../
