#include <ros/ros.h>
#include "ros_rnet_controller/JoyRNetController.hpp"

int main(int argc, char** argv) {
	ros::init(argc, argv, "joy_controller");

	rosrnet::JoyRNetController controller;
	
	ros::spin();
}
