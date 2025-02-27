
- https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo



# Requirements

- Ubuntu 22.04
- CPU 6cores and 12 thread or higher --> 12 Cores and 20 threads
- GPU RTX2080Ti or higher --> RTX 3080
- Nvidia Driver (Windows)>=472.50
- Nvidia Driver (Ubuntu 22)>=515.43.04 --> 535
- ROS2 Humble installed

# Network settings

- The simulation is based on the appropriate network setting, which allows for trouble-free communication of the AWSIM simulation with the Autoware software. To apply required localhost settings please add the following lines to `~/.bashrc` file.

> [!warning]
> This will result in a request to enter the password for sudo in every terminal start

```bash
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp 
if [ ! -e /tmp/cycloneDDS_configured ]; then
	sudo sysctl -w net.core.rmem_max=2147483647
	sudo ip link set lo multicast on
	touch /tmp/cycloneDDS_configured
fi
```

# Installation

## AWSIM

- https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/#start-the-demo

1. Install Nvidia GPU driver (Skip if already installed).
   1. Add Nvidia driver to apt repository

```bash
sudo add-apt-repository ppa:graphics-drivers/ppa
```

```bash
sudo apt update
```
2. Install the recommended version of the driver.
```bash
sudo ubuntu-drivers autoinstall
```
3. Reboot your machine to make the installed driver detected by the system. 
```bash
sudo reboot
```
4. Open terminal and check if `nvidia-smi` command is available and outputs summary similar to the one presented below. 
```bash
nvidia-smi
````

```bash
Sat Nov 25 13:21:51 2023       
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.129.03             Driver Version: 535.129.03   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 3080        Off | 00000000:01:00.0  On |                  N/A |
|  0%   34C    P8              11W / 370W |    202MiB / 10240MiB |      3%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
                                                                                         
+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|    0   N/A  N/A      1916      G   /usr/lib/xorg/Xorg                           73MiB |
|    0   N/A  N/A      2050      G   /usr/bin/gnome-shell                        120MiB |
+---------------------------------------------------------------------------------------+
```
        
2. Install Vulkan Graphics Library (Skip if already installed).
    
    1. Update the environment.
	```bash        
sudo apt update
	```
    2. Install the library.
    ```bash
sudo apt install libvulkan1
	```
3. Download and Run AWSIM Demo binary.
    1. Download `AWSIM_v1.1.0.zip`.
        [Download AWSIM Demo for ubuntu](https://github.com/tier4/AWSIM/releases/download/v1.1.0/AWSIM_v1.1.0.zip)
    2. Unzip the downloaded file.
    3. Make sure the `AWSIM_demo.x86_64` file is executable.
        - Rightclick the `AWSIM_demo.x86_64` file and check the `Execute` checkbox if not already
        ![](https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/Image_1.png)
       -  or execute the command below.
        ```bash
chmod +x <path to AWSIM folder>/AWSIM_demo.x86_64
        ```


## Autoware for AWSIM
- https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo/#launching-autoware

In order to configure and run the Autoware software with the AWSIM demo, please:

1. Download `map files (pcd, osm)` and unzip them.
    [Download Map files (pcd, osm)](https://github.com/tier4/AWSIM/releases/download/v1.1.0/nishishinjuku_autoware_map.zip)
2. Follow [[Environment_Autoware-Source_Ubuntu22-04]] 
	- AWSIM Quickstart manual stated to switch branch. When trying it, the build failed with [[Environment_AWSIM_QuickStartDemo#trtexec_vendor - mrm_comfortable_stop_operator]]
	- Before `mkdir src` Switch branch to `awsim-stable`. _NOTE: The latest `main` branch is for [ROS 2 humble](https://docs.ros.org/en/rolling/Releases/Release-Humble-Hawksbill.html).
	    ```bash
		git checkout awsim-stable
		```

3. Build the workspace.
	```bash
	colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="-w"
	```
> [!caution]
>When using this build command, it was a not able to build.



# Launch Simulation
## AWSIM
- Launch `AWSIM_demo.x86_64`.

```bash
source /opt/ros/humble/setup.bash
````

```bash
./<path to AWSIM folder>/AWSIM_demo.x86_64
```

```bash
~/AWSIM/AWSIM_v1.1.0/AWSIM_demo.x86_64 
```

> [!warning]
> It may take some time for the application to start the so please wait until image is visible in your application window.

## Autoware for AWSIM

- Launch Autoware.

> [!warning]
> `<your mapfile location>` must be changed arbitrarily. When specifying the path the `~` operator cannot be used - please specify absolute full path.

- source ros2 environment

```bash
source install/setup.bash
```

- launch autoware

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=<your mapfile location>
```

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:="/home/andrew/AWSIM/nishishinjuku_autoware_map"
```

```bash
ros2 launch autoware_launch e2e_simulator.launch.xml vehicle_model:=sample_vehicle sensor_model:=awsim_sensor_kit map_path:=/home/andrew/AWSIM/AWSIM_map
```

## Start Vehicle

- open terminal and go into the `autoware` directory

```bash
cd autoware
```

- source ros2
```bash
source install/setup.bash
```

- publish topic to start the navigation
```bash
ros2 topic pub /autoware/engage autoware_auto_vehicle_msgs/msg/Engage '{engage: True}' -1
```

- https://blog.tleyden.net/get-awsim-autoware-simulator-running-on-ubuntu-20-04/

# Errors

## src exists

```bash
andrew@andrew-Z690-UD:~/autoware$ mkdir src vcs import src < autoware.repos
mkdir: cannot create directory ‘src’: File exists
```

\--> continued anyway
\--> Those are actually two commands --> updated docu and started from this point again

## humble/setup.bash no such file or directory

```bash
andrew@andrew-Z690-UD:~/autoware$ source /opt/ros/humble/setup.bash rosdep update rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
bash: /opt/ros/humble/setup.bash: No such file or directory
```

\--> Install ROS2 Humble \[\[Environment_ROS2-humble_Ubuntu22-04\]\]

## pacmod_interfaces missing

```bash
ERROR: the following packages/stacks could not have their rosdep keys resolved
to system dependencies:
pacmod_interface: Cannot locate rosdep definition for [pacmod3_msgs]
```

\--> continued anyway

### install pacmod3_msgs

- https://github.com/astuff/pacmod3_msgs/tree/main

```
sudo apt install apt-transport-https
sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
sudo apt update
sudo apt install ros-$ROS_DISTRO-pacmod3-msgs  
```

\--> solved

## velodny driver

- when building autoware

```bash
Starting >>> autoware_point_types
--- stderr: velodyne_driver                                                                                                                                            
In file included from /home/andrew/autoware/src/sensor_component/external/velodyne_vls/velodyne_driver/src/lib/input.cc:34:
/home/andrew/autoware/src/sensor_component/external/velodyne_vls/velodyne_driver/include/velodyne_driver/input.h:37:10: fatal error: pcap.h: No such file or directory
   37 | #include <pcap.h>
      |          ^~~~~~~~
compilation terminated.
gmake[2]: *** [src/lib/CMakeFiles/velodyne_input.dir/build.make:76: src/lib/CMakeFiles/velodyne_input.dir/input.cc.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:208: src/lib/CMakeFiles/velodyne_input.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< velodyne_driver [2.93s, exited with code 2]
```

### install libcyp-dev

- https://github.com/projectdiscovery/naabu/issues/125

```bash
sudo apt install -y libpcap-dev
```

- Make sure to clean up your local ros workspace (i.e. remove build/ install/ log/) before `colcon build`. --> solved

## rtc_interface

```bash
Starting >>> rtc_interface                                                                                                                                                                     
--- stderr: tamagawa_imu_driver                                                                                                                                                                
/home/andrew/autoware/src/sensor_component/external/tamagawa_imu_driver/src/tag_can_driver.cpp:49:10: fatal error: can_msgs/msg/frame.hpp: No such file or directory
   49 | #include "can_msgs/msg/frame.hpp"
      |          ^~~~~~~~~~~~~~~~~~~~~~~~
compilation terminated.
gmake[2]: *** [CMakeFiles/tag_can_driver.dir/build.make:76: CMakeFiles/tag_can_driver.dir/src/tag_can_driver.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:165: CMakeFiles/tag_can_driver.dir/all] Error 2
gmake[1]: *** Waiting for unfinished jobs....
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< tamagawa_imu_driver [1.23s, exited with code 2]
```

### install humble-can-msg

- https://answers.ros.org/question/375690/fatal-error-can_msgsmsgframehpp-no-such-file-or-directory/

```bash
sudo apt install ros-$ROS_DISTRO-can-msgs
```

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
```

- Make sure to clean up your local ros workspace (i.e. remove build/ install/ log/) before `colcon build`. --> did not solve

## 'point_cloud_msg_wrapper' was not found before

```bash
Starting >>> autoware_auto_planning_msgs
--- stderr: autoware_point_types                                                                                                                                    
CMake Error at /opt/ros/humble/share/ament_cmake_target_dependencies/cmake/ament_target_dependencies.cmake:77 (message):
  ament_target_dependencies() the passed package name
  'point_cloud_msg_wrapper' was not found before
Call Stack (most recent call first):
  CMakeLists.txt:20 (ament_target_dependencies)


---
Failed   <<< autoware_point_types [1.71s, exited with code 1]
```

\--> not solved

## cudnn etc. not installed

- script for installing dependencies was not working
- install manually all dependencies

## trtexec_vendor - tier4_perception_msgs

```bash
Starting >>> tier4_perception_msgs
--- stderr: trtexec_vendor                                                                                                                                                                            
[ 11%] Creating directories for 'tensorrt-populate'
[ 22%] Performing download step (git clone) for 'tensorrt-populate'
Cloning into 'tensorrt-src'...
fatal: invalid reference: 8.6.1
CMake Error at tensorrt-subbuild/tensorrt-populate-prefix/tmp/tensorrt-populate-gitclone.cmake:40 (message):
  Failed to checkout tag: '8.6.1'


gmake[2]: *** [CMakeFiles/tensorrt-populate.dir/build.make:102: tensorrt-populate-prefix/src/tensorrt-populate-stamp/tensorrt-populate-download] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:83: CMakeFiles/tensorrt-populate.dir/all] Error 2
gmake: *** [Makefile:91: all] Error 2

CMake Error at /usr/share/cmake-3.22/Modules/FetchContent.cmake:1087 (message):
  Build step for tensorrt failed: 2
Call Stack (most recent call first):
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1216:EVAL:2 (__FetchContent_directPopulate)
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1216 (cmake_language)
  CMakeLists.txt:45 (fetchcontent_populate)


---
Failed   <<< trtexec_vendor [8.41s, exited with code 1]
Aborted  <<< livox_tag_filter [5.73s] 
```

### clone autoware repo with -- recursive

\--> did not solve

## trtexec_vendor - localization_error_monitor

```bash
Starting >>> localization_error_monitor
--- stderr: trtexec_vendor                                                                                                                                                                           
[ 11%] Creating directories for 'tensorrt-populate'
[ 22%] Performing download step (git clone) for 'tensorrt-populate'
Cloning into 'tensorrt-src'...
fatal: invalid reference: 8.6.1
CMake Error at tensorrt-subbuild/tensorrt-populate-prefix/tmp/tensorrt-populate-gitclone.cmake:40 (message):
  Failed to checkout tag: '8.6.1'


gmake[2]: *** [CMakeFiles/tensorrt-populate.dir/build.make:102: tensorrt-populate-prefix/src/tensorrt-populate-stamp/tensorrt-populate-download] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:83: CMakeFiles/tensorrt-populate.dir/all] Error 2
gmake: *** [Makefile:91: all] Error 2

CMake Error at /usr/share/cmake-3.22/Modules/FetchContent.cmake:1087 (message):
  Build step for tensorrt failed: 2
Call Stack (most recent call first):
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1216:EVAL:2 (__FetchContent_directPopulate)
  /usr/share/cmake-3.22/Modules/FetchContent.cmake:1216 (cmake_language)
  CMakeLists.txt:45 (fetchcontent_populate)


---
Failed   <<< trtexec_vendor [7.48s, exited with code 1]
```

- https://github.com/autowarefoundation/autoware.universe/issues/1893
- it seems the tag *8\.6.1* is not existing but only tag *v8.6.1* inserted after line 37 in `/src/universe/autoware.universe/common/trtexec_vendor/CMakeLists.txt`

```txt
# added manually because of build error
  set(TENSORRT_VERSION 8.5.1)
```

## trtexec_vendor - mrm_comfortable_stop_operator

```bash
Starting >>> mrm_comfortable_stop_operator
--- stderr: trtexec_vendor                                                                                                                                                                                 
In file included from /home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/common/common.h:60,
                 from /home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/common/buffers.h:21,
                 from /home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/trtexec/trtexec.cpp:36:
/home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/common/safeCommon.h: In member function ‘void samplesCommon::TrtCudaGraphSafe::endCapture(CUstream_st*&)’:
/home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/common/safeCommon.h:155:57: error: cannot convert ‘std::nullptr_t’ to ‘long long unsigned int’
  155 |         CHECK(cudaGraphInstantiate(&mGraphExec, mGraph, nullptr, nullptr, 0));
      |                                                         ^~~~~~~
      |                                                         |
      |                                                         std::nullptr_t
/home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/common/safeCommon.h:43:21: note: in definition of macro ‘CHECK’
   43 |         auto ret = (status);                                                                                           \
      |                     ^~~~~~
In file included from /usr/include/x86_64-linux-gnu/NvInferRuntimeBase.h:19,
                 from /usr/include/x86_64-linux-gnu/NvInferRuntimeCommon.h:26,
                 from /usr/include/x86_64-linux-gnu/NvInferLegacyDims.h:16,
                 from /usr/include/x86_64-linux-gnu/NvInfer.h:16,
                 from /home/andrew/autoware/build/trtexec_vendor/_deps/tensorrt-src/samples/trtexec/trtexec.cpp:33:
/usr/local/cuda/include/cuda_runtime_api.h:12021:127: note:   initializing argument 3 of ‘cudaError_t cudaGraphInstantiate(CUgraphExec_st**, cudaGraph_t, long long unsigned int)’
12021 | extern __host__ cudaError_t CUDARTAPI cudaGraphInstantiate(cudaGraphExec_t *pGraphExec, cudaGraph_t graph, unsigned long long flags __dv(0));
      |                                                                                                                               ^
gmake[2]: *** [CMakeFiles/trtexec_vendor.dir/build.make:76: CMakeFiles/trtexec_vendor.dir/_deps/tensorrt-src/samples/trtexec/trtexec.cpp.o] Error 1
gmake[1]: *** [CMakeFiles/Makefile2:137: CMakeFiles/trtexec_vendor.dir/all] Error 2
gmake: *** [Makefile:146: all] Error 2
---
Failed   <<< trtexec_vendor [9.63s, exited with code 2]
Aborted  <<< mrm_comfortable_stop_operator [0.13s]
```

### do not use `awsim-stable` branch

\--> helped

## segmentaiton fault (core dump)

### install newer driver

- check nvidia driver in action

  ```bash
  nvidia-smi
  ```
- Required: Nvidia Driver (Ubuntu 22)>=515.43.04
- https://ubuntu.com/server/docs/nvidia-drivers-installation
- get all available drivers on the system

  ```bash
  sudo ubuntu-drivers list
  ```
- if the driver is not installed at all go back to the tutorials
- else install the driver you need (here an example)

  ```bash
  sudo ubuntu-drivers install nvidia:535
  ```

\--> did not help

### replace the AWSIM file with new downloaded file

\--> did not help

### add lines to .bashrc, update and install libvulkan1

- it is unclear which of those helped or all together, but it is working again
- add lines to `.bashrc` as described in the manual
- make update

```bash
sudo apt update
```

- install libvulkan1 again

```bash
	sudo apt install lubvulkan1
```

\--> helped

## BadValue (integer parameter out of range for operation)

```bash
andrew@andrew-Z690-UD:~/autoware$ ~/AWSIM/AWSIM_v1.1.0/AWSIM_demo.x86_64 
Found path: /home/andrew/AWSIM/AWSIM_v1.1.0/AWSIM_demo.x86_64
X Error of failed request:  BadValue (integer parameter out of range for operation)
  Major opcode of failed request:  151 (GLX)
  Minor opcode of failed request:  3 (X_GLXCreateContext)
  Value in failed request:  0x0
  Serial number of failed request:  96
  Current serial number in output stream:  97
```

### restart

\--> helped


# when drivers break

```bash
nvidia-smi
```

```bash
sudo ubuntu-drivers list --gpgpu
```

```bash
sudo ubuntu-drivers install nvidi:535
```

```bash
sudo apt install nvidia-driver-535
```

```bash
sudo apt install libnvidia-compute-535
```

```bash
nvidia-smi
```

```bash
sudo apt install nvidia-driver-535
```

```bash
sudo apt install libnvidia-fbc1-535
```

```bash
sudo apt install nvidia-driver-535
```

```bash
restart
```

```bash
nvidia-smi
```