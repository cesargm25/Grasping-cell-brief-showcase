#!/bin/bash

xhost +local:docker || true

ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
echo $ROOT_DIR
if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker run --gpus all \
                -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v /dev:/dev \
               --mount source=ws_moveit,target=/ws_moveit \
               --net=host \
               --privileged \
               --name cg3yclonedds cg3yclonedds-img
               bash -i -c "source /ws_moveit/install/setup.bash; exec bash"

else

    echo "[!] If you wanna use nvidia gpu, please use script with -n or --nvidia argument"
    docker run  -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v /dev:/dev \
                -v ./ws_moveit:/ws_moveit \
               --net=host \
               --privileged \
               --name cg3yclonedds cg3yclonedds-img
               bash -i -c "source /ws_moveit/install/setup.bash; exec bash"
fi
