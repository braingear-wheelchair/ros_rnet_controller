#ifndef ROS_JOY_RNETCONTROLLER_CPP
#define ROS_JOY_RNETCONTROLLER_CPP

#include "ros_rnet_controller/JoyRNetController.hpp"

namespace rosrnet {
JoyRNetController::JoyRNetController(void)  {

	   this->linear_ = 1;
	   this->angular_ = 0;

	   this->a_scale_ = 0.27f;
	   this->l_scale_ = 0.27f;
   
     	nh_.param("axis_linear", linear_, linear_);
     	nh_.param("axis_angular", angular_, angular_);
     	nh_.param("scale_angular", a_scale_, a_scale_);
     	nh_.param("scale_linear", l_scale_, l_scale_);

		vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
  		joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &JoyRNetController::on_received_joy, this);
}

JoyRNetController::~JoyRNetController(void) {}

void JoyRNetController::on_received_joy(const sensor_msgs::Joy::ConstPtr& joy) {
	geometry_msgs::Twist twist;
  	twist.angular.z = a_scale_*joy->axes[angular_];
  	twist.linear.x = l_scale_*joy->axes[linear_];
  
	this->vel_pub_.publish(twist);
}

}

#endif
