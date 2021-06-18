#include <ros/ros.h>
#include <rnetserial/RNetSerial.hpp>
#include "ros_rnet_controller/RNetController.hpp"

#define DEFAULT_RNET_SERIALPORT "/dev/ttyUSB0"

int main (int argc, char** argv) {

	ros::init(argc, argv, "rnet_controller");

	std::string port;
	rnet::RNetSerial rnet;
	rosrnet::RNetController controller(&rnet);

	/*** Gathering parameters ***/
	ros::param::param<std::string>("~port", port, DEFAULT_RNET_SERIALPORT);

	/*** Opening serial port ***/
	if(rnet.Open(port) == false) {
		ROS_ERROR("[%s] Serial port \"%s\" is NOT open.\n", rnet.name().c_str(), port.c_str());
		return EXIT_FAILURE;
	}
	ROS_INFO("[%s] Serial port %s is open and set with default parameters.\n", 
			 rnet.name().c_str(), port.c_str());

	/*** Connecting to the chipset ***/
	ROS_INFO("[%s] Establishing connection to Rebus Chipset...\n", rnet.name().c_str());	
	if(rnet.Connect() == false) {
		ROS_ERROR("[%s] Cannot establish connection with Rebus Chipset.\n", rnet.name().c_str());
		return EXIT_FAILURE;
	}
	ROS_INFO("[%s] Connection established.\n", rnet.name().c_str());

	/*** Configuring the controller ***/
	if(controller.Configure() == false) {
		ROS_ERROR("[%s] Cannot configure the controller.\n", rnet.name().c_str());
	}
	ROS_INFO("[%s] Controller configured.\n", rnet.name().c_str());
	
	
	/*** Starting the controller (blocking) ***/
	ROS_INFO("[%s] Starting the controller.\n", rnet.name().c_str());
	controller.Run();


	/*** Closing the connection ***/
	ROS_INFO("[%s] Closing rnet connection.\n", rnet.name().c_str());
	rnet.Close();

	ros::shutdown();

	return 0;
}
