#!/bin/bash

source ~/.bashrc

if [ -d ./src/quad-sdk ]
then
    DIR_NAME=quad-sdk
else
    DIR_NAME=Legged_robotics
fi



cd ./src/$DIR_NAME/external/ipopt/coinbrew/Ipopt/
./configure
make
make install

cd ../ThirdParty/HSL
./configure
make
make install

cd ./../../../../../../../

./src/$DIR_NAME/setup.sh
catkin build

apt update
apt -y upgrade 

source ~/.bashrc
source ~/../catkin_ws/devel/setup.bash

ln -s /usr/local/lib/libcoinhsl.so /usr/local/lib/libhsl.so


