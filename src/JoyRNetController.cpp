#ifndef ROS_JOY_RNETCONTROLLER_CPP
#define ROS_JOY_RNETCONTROLLER_CPP

#include "ros_rnet_controller/JoyRNetController.hpp"

namespace rosrnet {
JoyRNetController::JoyRNetController(void)  {

	   this->linear_ = 1;
	   this->angular_ = 0;

	   this->max_forward_velocity_  = 0.27f;
	   this->max_backward_velocity_ = 0.27f;
	   this->max_turning_velocity_  = 0.27f;
   
     	nh_.param("axis_linear", linear_, linear_);
     	nh_.param("axis_angular", angular_, angular_);
     	nh_.param("max_forward", this->max_forward_velocity_, this->max_forward_velocity_);
     	nh_.param("max_backward", this->max_backward_velocity_, this->max_backward_velocity_);
     	nh_.param("max_turning", this->max_turning_velocity_, this->max_turning_velocity_);

		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyRNetController::on_received_joy, this);
}

JoyRNetController::~JoyRNetController(void) {}

void JoyRNetController::on_received_joy(const sensor_msgs::Joy::ConstPtr& joy) {
	geometry_msgs::Twist twist;
  	twist.angular.z = this->normalize(joy->axes[angular_], this->max_turning_velocity_, -this->max_turning_velocity_);
  	twist.linear.x = this->normalize(joy->axes[linear_], this->max_forward_velocity_, -this->max_backward_velocity_);
  
	this->vel_pub_.publish(twist);
}

float JoyRNetController::normalize(float value, float max_allowed, float min_allowed) {

	float max, min;
	
	
	max = 1.0f;
	min = -1.0f;


	return ( ( ( (max_allowed - min_allowed) * (value - min) ) / (max - min) ) + min_allowed);
}

}

#endif
