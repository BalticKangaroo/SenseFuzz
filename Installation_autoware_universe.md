
# Install autoware from source
- https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/



# Dependencies
- Ubuntu 22.04
- git
- https://docs.nvidia.com/datacenter/tesla/tesla-installation-notes/index.html


# Install Dependencies
1. ROS2 Humble [[Environment_ROS2-humble_Ubuntu22-04]]
2. Install ros2_dev_tools
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/ros2_dev_tools#manual-installation
	```bash
	sudo apt update && sudo apt install -y \
	  python3-flake8-docstrings \
	  python3-pip \
	  python3-pytest-cov \
	  ros-dev-tools
	```

	```bash
	sudo apt install -y \
	  python3-flake8-blind-except \
	  python3-flake8-builtins \
	  python3-flake8-class-newline \
	  python3-flake8-comprehensions \
	  python3-flake8-deprecated \
	  python3-flake8-import-order \
	  python3-flake8-quotes \
	  python3-pytest-repeat \
	  python3-pytest-rerunfailures
		```
	-  Initialize rosdep
	```bash
	sudo rosdep init
	rosdep update
	```
3. Install rmw_implementation
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/rmw_implementation#manual-installation
	```bash
	wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
	```
	- For details: https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html
	```bash
	sudo apt update
	rmw_implementation_dashed=$(eval sed -e "s/_/-/g" <<< "${rmw_implementation}")
	sudo apt install ros-${rosdistro}-${rmw_implementation_dashed}
	```
	- (Optional) You set the default RMW implementation in the ~/.bashrc file.
	```bash
	echo '' >> ~/.bashrc && echo "export RMW_IMPLEMENTATION=${rmw_implementation}" >> ~/.bashrc
	```
4. install pacmod
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/pacmod#manual-installation
	```bash
	wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
	```
	- Taken from https://github.com/astuff/pacmod3#installation
	```bash
	sudo apt install apt-transport-https
	sudo sh -c 'echo "deb [trusted=yes] https://s3.amazonaws.com/autonomoustuff-repo/ $(lsb_release -sc) main" > /etc/apt/sources.list.d/autonomoustuff-public.list'
	sudo apt update
	sudo apt install ros-${rosdistro}-pacmod3
	```
5. install autoware_core dependencies
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/autoware_core#manual-installation
	- Install gdown to download files from CMakeLists.txt
	```bash
pip3 install gdown
	```
6. install autoware_universe dependencies
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/autoware_universe#manual-installation
	```bash
	sudo apt install geographiclib-tools
	```
	- Add EGM2008 geoid grid to geographiclib
	```bash
	sudo geographiclib-get-geoids egm2008-1
	```
7. install pre_commit dependencies
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/pre_commit#manual-installation
	```bash
	pre_commit_clang_format_version=17.0.5
	pip3 install pre-commit clang-format==${pre_commit_clang_format_version}
	```

	```bash
	sudo apt install golang
	```
8. install nvidia cuda
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/cuda#manual-installation
	- find architecture with:
	```bash
	dpkg --print-architecture
	```
	
	```bash
	wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
	```
	- From: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#network-repo-installation-for-ubuntu
	```bash
	os=ubuntu2204
	wget https://developer.download.nvidia.com/compute/cuda/repos/$os/$(uname -m)/cuda-keyring_1.1-1_all.deb
	sudo dpkg -i cuda-keyring_1.1-1_all.deb
	sudo apt-get update
	cuda_version_dashed=$(eval sed -e "s/[.]/-/g" <<< "${cuda_version}")
	sudo apt-get -y install cuda-${cuda_version_dashed}
	```
	- **RESTART**
	- perform post installation actions
	- Taken from: https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html#post-installation-actions
	```bash
	echo 'export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}' >> ~/.bashrc
	echo 'export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}' >> ~/.bashrc
	```
9. install cudnn and tensorrt
	- https://github.com/autowarefoundation/autoware/tree/main/ansible/roles/tensorrt#manual-installation
	- For Universe, the `cudnn_version` and `tensorrt_version` variables should be copied from [amd64.env](https://github.com/autowarefoundation/autoware/blob/main/amd64.env) or [arm64.env](https://github.com/autowarefoundation/autoware/blob/main/arm64.env) depending on the architecture used.
	- find architecture with:
	```bash
	dpkg --print-architecture
	```

	```bash
	wget -O /tmp/amd64.env https://raw.githubusercontent.com/autowarefoundation/autoware/main/amd64.env && source /tmp/amd64.env
	```
	- Taken from: https://docs.nvidia.com/deeplearning/tensorrt/install-guide/index.html#installing
	```bash
	sudo apt-get install libcudnn8=${cudnn_version} libcudnn8-dev=${cudnn_version}
	sudo apt-mark hold libcudnn8 libcudnn8-dev
	```

	```bash
	sudo apt-get install libnvinfer8=${tensorrt_version} libnvonnxparsers8=${tensorrt_version} libnvparsers8=${tensorrt_version} libnvinfer-plugin8=${tensorrt_version} libnvinfer-dev=${tensorrt_version} libnvonnxparsers-dev=${tensorrt_version} libnvparsers-dev=${tensorrt_version} libnvinfer-plugin-dev=${tensorrt_version}
	sudo apt-mark hold libnvinfer8 libnvonnxparsers8 libnvparsers8 libnvinfer-plugin8 libnvinfer-dev libnvonnxparsers-dev libnvparsers-dev libnvinfer-plugin-dev
	```


# Install Autoware
- https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-set-up-a-development-environment
- https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-set-up-a-workspace

1. Clone `autowarefoundation/autoware` and move to the directory.
    ```bash
    git clone --recursive https://github.com/autowarefoundation/autoware.git
    cd autoware
	```

> [!caution] 
> For usage with AWSIM simulator, switch to other branch here
> AWSIM Quickstart manual stated to switch branch. When trying it, the build failed with [[Environment_AWSIM_QuickStartDemo#trtexec_vendor - mrm_comfortable_stop_operator]]
```bash
git checkout awsim-stable
``` 

2. Create the `src` directory and clone repositories into it.
    - Autoware uses [vcstool](https://github.com/dirk-thomas/vcstool) to construct workspaces.
    ```bash
    mkdir src
    ```
    
	```bash
    vcs import src < autoware.repos
	```
3. Install dependent ROS packages.
    
    - Autoware requires some ROS 2 packages in addition to the core components. The tool `rosdep` allows an automatic search and installation of such dependencies. 
	```bash
	source /opt/ros/humble/setup.bash
	```
	- You might need to run `rosdep update` before `rosdep install`.
	```bash
	rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
    ```
4. Build the workspace.
	- Autoware uses [colcon](https://github.com/colcon) to build workspaces. For more advanced options, refer to the [documentation](https://colcon.readthedocs.io/).
```bash
    colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> [!caution] 
> Building for AWSIM setup differs. 


# Start Autoware

```bash
cd autoware
```

```bash
source install/setup.bash
```

```bash
ros2 launch autoware_launch planning_simulator.launch.xml map_path:=$HOME/autoware_map/sample-map-planning vehicle_model:=sample_vehicle sensor_model:=sample_sensor_kit

```

