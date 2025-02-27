# ROS2 Fuzzing Broker

This README should briefly explain how to start the current implementation. 

The carla-ros-bridge is not part of the Fuzzing Broker. It contains modifications for a better usage. It was depreciated earlier in the developement process and only has limited comments. It can be found in the *carla-ros-bridge* branch.

The branch *AWSIM_fuzzing-broker* has the status of the code when the thesis was handed in.

The branch *fuzzing-broker* very likely has a running code base for a connection to carla with the ros bridge.

## Content
[**Requirements**](#requirements)

[**Preparements**](#preparements)

[**Start**](#start)

[**Basic Configuration**](#basic-configuration)

[**License**](#license)

[**Maintainer**](#maintainer)

## Requirements
- Ubuntu 22.04 (Desktop version)
- ROS2 Humble
- Python 3.10.12
    - rclpy
- AWSIM (on machine)
    - https://tier4.github.io/AWSIM/GettingStarted/QuickStartDemo
    - There is a file called Installation_AWSIM_QuickstartDemo* in this repository. It might give solutions for issues. However, the installation is dependent on the machine and might differ.
- autoware.universe (on machine)
    - https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/
    - There is a file called *Installation_autoware_universe* in this repository. It might give solutions for issues. However, the installation is dependent on the machine and might differ.

## Preparements
- Ensure the values for the parameters in the fuzzingparams.yml are correct
    - Topic the fuzzing broker should subscribe to (for AWSIM it might be already correct. Otherwise check for example with RQT)
    - choose topic the manipulated data should be published (ensure the ADS can subscribe to it)
    - choose your fuzzing parameters

- For autoware.universe it can be complex to change the topics it subscribes to. A workaround is opening the autoware.universe directory on the machine after installation and change "pointcloud_raw_ex" to "pointcloud_raw_ex_fuzzed" in all files and ensure it is also published like this from the broker.

- Further explanations can be found in 

## Start
- Start AWSIM
- Start autoware.universe
- Start the fuzzing brokers
1. Go into the `ros_fuzzing_broker` directory
2. Source ROS
```bash
source /opt/ros/humble/setup.bash
````
3. if not already build, buld the package,
Build with a symlink to not require rebuild while developing
```bash
colcon build --packages-select ros2_fuzzing_broker --symlink-install
```
*--> requires the python files in the package to be executables*
```bash
chmod +x <filename>.py
```

4. Source the ROS2 package
```bash
source install/setup.bash
````
5. run the node
```bash
ros2 run <packageName> <nodeName>
````
Example:
```bash
ros2 run ros2_fuzzing_broker awsim_fuzzingbroker_lidar
````

## Basic Configuration
A basic configuration of the parameters is possible in the fuzzingparams.yml file in "ros_fuzzing_broker/fuzzingparams.yml"

## License
ROS2 Fuzzing Broker is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.

ROS2 Fuzzing Broker is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with Foobar. If not, see <https://www.gnu.org/licenses/>.


