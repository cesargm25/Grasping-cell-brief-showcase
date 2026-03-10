
# Grasping Pipeline Setup

Compact setup guide for the grasping pipeline using:

- **UR robot + MoveIt2**
- **Intel RealSense depth camera**
- **YOLO ROS**
- **Contact-GraspNet**
- **Robotiq gripper**
- optional TF mounting and custom pose node

---

## Overview

This pipeline uses:

- **RealSense ROS** to publish RGB-D camera data
- **YOLO ROS** to detect / segment objects
- **Contact-GraspNet** to generate grasp candidates from depth / point cloud input
- **UR ROS2 Driver + MoveIt2** to move the robot
- **Robotiq repo** for gripper support
- **custom TF / pose scripts** to connect camera and grasp frames

---

## Main Notes

- First install the dependencies from the original repository requirements file, usually `requirements.txt`
- Then run the camera
- Then run Contact-GraspNet with the input coming from the depth camera
- If you want to use **GPU**, it must be configured for GPU execution
- Do **not** run the same setup simultaneously in CPU mode and GPU mode

---

## Important Camera Setting

Depth alignment must be enabled on the camera:

```bash
ros2 param set /camera/camera align_depth.enable true
````

This is required so the depth data matches the color image correctly.

---

## Check / Enable Depth Alignment

```bash
ros2 topic list | grep aligned
ros2 param list /camera/camera | grep -i align
ros2 param set /camera/camera align_depth.enable true
```

---

## Dependencies

### Required repositories

* **Robotiq**
  Used for the gripper interface and gripper control.

* **IFRA_LinkAttacher**
  Used in simulation to attach / detach objects during grasp tests.

* **realsense-ros**
  Used to publish RGB, depth, point cloud, and camera topics from the Intel RealSense camera.

* **Contact-GraspNet**
  Used to generate 6-DoF grasp candidates from depth / point cloud data.

* **Universal_Robots_ROS2_Driver**
  Used to connect and control the UR robot in ROS2.

* **yolo_ros**
  Used for object detection / segmentation.
  **Important:** all internal dependencies and installation steps from the original `yolo_ros` repository must also be installed.

---

## Installation Notes

### 1. Install repository dependencies

Install:

* system dependencies
* ROS2 dependencies
* Python dependencies from each repo
* especially the `requirements.txt` from the original Contact-GraspNet repo

### 2. Camera input

Start the RealSense camera and verify:

* RGB image is published
* depth image is published
* aligned depth is enabled
* point cloud is available if needed

### 3. Contact-GraspNet input

Run Contact-GraspNet using the depth camera output as input.

### 4. CPU / GPU note

If changing the setup to GPU execution, do not keep the CPU version running at the same time.

---

## Launch Sequence
The commands to run the necessary lunch files are reduced into bash files for an easier operation. 

### A. Test Docker

#### Terminal 1

```bash
ws_moveit/Moving/./ur_code.sh
```

or without hardware:

```bash
ws_moveit/Moving/./ur_simulation.sh
```

#### Terminal 2

```bash
ws_moveit/Moving/./cus_moveit.sh
```

#### Terminal 3

```bash
ws_moveit/folder_cv/./frame_camera.sh
```

> Note: this frame could also be added directly into the URDF.

---

### B. Grasping Docker

#### Terminal 1

```bash
ws_moveit/./camera.sh
```

#### Terminal 2

```bash
/ws_moveit/folder_cv/./profun.sh
```

#### Terminal 3

```bash
/ws_moveit/folder_cv/./mount_camera.sh
```

Then publish the static transform:

```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 3.14159265 0 0 camera_mount_block camera_link
```

#### Terminal 4

```bash
python3 tfpose.py
```

> Note: `tfpose.py` is not yet inside a ROS2 package.
> It should be converted into a proper ROS2 node inside a package.

---

## Current Structure

### Camera

Publishes:

* RGB image
* depth image
* aligned depth
* camera frames

### YOLO ROS

Provides:

* object detection
* segmentation / target localization

### Contact-GraspNet

Uses:

* depth / point cloud input
* target region if needed
* grasp generation

### Robot / MoveIt2

Handles:

* motion planning
* pre-grasp / grasp / lift motion
* execution on the UR robot

### Gripper

Handles:

* open / close commands
* object grasp execution
* Object detection missing 

---

## Recommended Setup Checks

Before running the full pipeline, verify:

* camera is publishing correctly
* aligned depth is enabled
* robot driver is running
* MoveIt2 is running
* camera TF is available
* static transform is correct
* YOLO ROS is running
* Contact-GraspNet dependencies are installed
* `tfpose.py` is available or converted into a node
* gripper interface is working

---

## Pending Improvements

* Move `tfpose.py` into a ROS2 package as a node
* Move camera frame setup into URDF if possible
* unify launch scripts into one launch file
* document Contact-GraspNet input topic / interface more clearly
* define CPU vs GPU execution in a cleaner way

---

## Repositories
This setup builds on the following open-source projects:

* **[Robotiq](https://github.com/ros-industrial-attic/robotiq)**
  for getting the geometry of the Gripper

  
* **[Contact-GraspNet](https://github.com/NVlabs/contact_graspnet)**
  Grasping dataset and grasping pose


* **[YOLO ROS](https://github.com/mgonzs13/yolo_ros)**
  Used for segmentation and classification 


* **[IFRA-Cranfield ROS 2 Robotiq Gripper Driver](https://github.com/IFRA-Cranfield/IFRA_LinkAttacher)**  
  Used to interface with the Robotiq 2F-140 gripper over Modbus TCP.

* **[Universal Robots ROS 2 Driver (UR Driver)](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)**  
  Official driver for controlling UR3e (and other UR series) robots using ROS 2.

* **[RealSense ROS 2 Wrapper (Intel)](https://github.com/realsenseai/realsense-ros)**  
   For integrating Intel RealSense cameras (e.g., D415 or D435).


