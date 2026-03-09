quiet_mkdir () {
	if ! [ -d $1 ]
	then
		mkdir $1
	fi
}

# Setup and build for rbdl
sudo apt install -y ros-noetic-urdf
cd rbdl-orb
quiet_mkdir build
cd build
cmake -D CMAKE_BUILD_TYPE=Release -D RBDL_BUILD_ADDON_URDFREADER=ON ..
make -j8
sudo make install
cd ../..

# Setup for teleop_twist_joy to get dependencies installed
sudo apt install -y ros-noetic-teleop-twist-joy
