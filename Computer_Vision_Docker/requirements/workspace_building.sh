#!/bin.bash

if ! [ -d ./../workspace/src/ ]
then
    mkdir -p ./../workspace/src/
fi

if ! [ -f ./../workspace/build.sh ]
then
    cp ./requirements/build.sh ./../workspace/build.sh
fi

if ! [ -d ./../workspace/src/quad-sdk ]
then
    cd ./../workspace/src/
    git clone -b noetic_devel https://github.com/robomechanics/quad-sdk.git
    #git clone -b go2_model git@github.com:Slavoch/Legged_robotics.git
else
    cd ./../workspace/src/
fi

if [ -d ./quad-sdk ]
then
    DIR_NAME=quad-sdk
else
    DIR_NAME=Legged_robotics
fi

echo $DIR_NAME

cp ./../../docker/requirements/setup_deps.sh ./$DIR_NAME/external/setup_deps.sh


######################################

cpp_file="./$DIR_NAME/nmpc_controller/src/nmpc_controller.cpp"

old_string='app_->Options()->SetStringValue("linear_solver", "ma27");'

new_string='app_->Options()->SetStringValue("linear_solver", "ma57");'

sed -i "s|$old_string|$new_string|g" "$cpp_file"

######################################


cd ./$DIR_NAME/external/ipopt/
mkdir -p coinbrew/ThirdParty
cd ./coinbrew/

if ! [ -d ./Ipopt ] 
then
    git clone -b stable/3.13 git@github.com:coin-or/Ipopt.git
fi

if ! [ -d ./ThirdParty-HSL ] 
then
    git clone git@github.com:coin-or-tools/ThirdParty-HSL.git
fi

cp -r ./ThirdParty-HSL/ ./ThirdParty/HSL/

if ! [ -d ./ThirdParty/HSL/coinhsl ]
then
    cp -r ./../../../../../../docker/requirements/coinhsl/ ./ThirdParty/HSL/coinhsl
fi

rm -rf ./ThirdParty-HSL/
