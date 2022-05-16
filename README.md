# ros_rnet_controller
The package provides a ROS wrapper to the controller for the RNet CAN bus of the wheelchair. It creates a ROS node that subscribes to the topic */cmd_vel* and send the received velocities to the RNet Chipset. 

## Requirements
ros_rnet_controller has been tested with the following configuration:
- **Ubuntu 18.04.05 LTS Bionic Beaver** and **ROS Melodic**
- **Ubuntu 20.04.02 LTS Focal Fossa** and **ROS Noetic**

ros_rnet_controller depends on the following libraries:
- [librnetserial](https://github.com/braingear-wheelchair/rnet-serial-interface) >= 0.1.0

### Subscribed Topics
- /cmd_vel

### Parameters
~<name>/`port` (`string`, default: /dev/ttyUSB0) [**mandatory**]
  
  The serial port connected to the RNet Chipset

~<name>/`profile` (`int`, default: 1) [optional]

  (Only profile 1 implemented with the following values: Forward velocity: 0.27 m/s; Backward velocity: 0.27 m/s; Angular velocity: 0.27 m/s)

~<name>/`maximum_forward_velocity` (`float`, default: 0.27) [optional]
  
  Maximum forward velocity, it overwrites the velocity in the profile
  
~<name>/`maximum_backward_velocity` (`float`, default: 0.27) [optional]
  
  Maximum backward velocity, it overwrites the velocity in the profile
  
~<name>/`maximum_turning_velocity` (`float`, default: 0.27) [optional]
  
  Maximum turning velocity, it overwrites the velocity in the profile
  
~<name>/`rate` (`int`, default: 100) [optional]
  
  Update rate of the internal loop (in Hz)
  
## Limitations
According to the RNet chipset requirements, the chipset needs to be restart (with hard button) every time the node is restarted.
  
## Usage in local setup
The node can be launched as follows:
  
``
  rosrun ros_rnet_controller rnet_controller _port:=/dev/ttyUSB0
``
  
## Usage over network
In the case the rnet_controller is running on a different machine directly connected to the wheelchair, it is possible to exploit ROS network in order to remotely launche the node. Below ww consider that the node is running in a raspberry pi 3 model B directly connected to the RNet Chipset via USB.
  
# ROS Network setup
We consider two machines:
  - *wheelchair-master* where the roscore is running
  - *wheelchair-pi* where the ros_rnet_controller node is running and directly connected to the RNet Chipset
  
The two machines are connected over Ethernet and they have been configured with the following IP:
  - *wheelchair-master*: 192.168.1.100
  - *wheelchair-pi*: 192.168.1.200
  
In *wheelchair-master* change the /etc/hosts file as follows:
  
```
  ...
  192.168.1.100	wheelchair-master
  192.168.1.200	wheelchair-pi
  ...
```
  
In *wheelchair-pi* change the /etc/hosts file as follows:
  
```
  ...
  192.168.1.100	wheelchair-master
  192.168.1.200	wheelchair-pi
  ...
```
  
In *wheelchair-master* sources the script *ros_set_variables.bash* in order to setup the ROS_MASTER_URI and the ROS_IP environment variables:
  ``
  source ros_set_variables.bash
  ``
  
Create a launch file 
