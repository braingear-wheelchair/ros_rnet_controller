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
		float normalize(float value, float max, float min);

	private:
		ros::NodeHandle nh_;
		int linear_, angular_;
     	ros::Publisher vel_pub_;
     	ros::Subscriber joy_sub_;

		float max_forward_velocity_;
		float max_backward_velocity_;
		float max_turning_velocity_;

};



}

#endif
