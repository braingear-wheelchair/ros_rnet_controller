#ifndef ROS_JOY_RNETCONTROLLER_HPP
#define ROS_JOY_RNETCONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

namespace rosrnet {

class JoyRNetController {

	public:
		JoyRNetController(void);
		virtual ~JoyRNetController(void);

	private:
		void on_received_joy(const sensor_msgs::Joy::ConstPtr& joy);

	private:
		ros::NodeHandle nh_;
		int linear_, angular_;
     	double l_scale_, a_scale_;
     	ros::Publisher vel_pub_;
     	ros::Subscriber joy_sub_;

};



}

#endif
