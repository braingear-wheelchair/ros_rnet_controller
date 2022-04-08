#ifndef ROS_RNETCONTROLLER_CPP
#define ROS_RNETCONTROLLER_CPP

#include "ros_rnet_controller/RNetController.hpp"

namespace rosrnet {

RNetController::RNetController(rnet::RNetSerial* serial) : p_nh_("~") {
	this->serial_ = serial;
	this->rx_	  = new rnet::RNetBuffer();
	this->tx_	  = new rnet::RNetBuffer();

	this->reader_ = new rnet::RNetReader(this->serial_, this->rx_); 
	this->writer_ = new rnet::RNetWriter(this->serial_, this->tx_); 
	this->srv_rx_ = new rnet::RNetServiceRx(this->tx_, this->rx_);
	this->srv_xy_ = new rnet::RNetServiceXY(this->tx_, this->rx_);

	this->topic_ = "/cmd_vel";

}

RNetController::~RNetController(void) {
	
	delete this->reader_;
	delete this->writer_;
	delete this->srv_rx_;
	delete this->srv_xy_;
	delete this->rx_;
	delete this->tx_;
}

bool RNetController::Configure(void) {

	int rate;
	int profile;
	ros::param::param<int>("~rate", rate, 100);
	ros::param::param<int>("~profile", profile, 1);
	
	this->set_velocity_profile(profile);

	ros::param::param<float>("~maximum_forward_velocity",  this->max_forward_velocity_, 
														   this->max_forward_velocity_);
	ros::param::param<float>("~maximum_backward_velocity", this->max_backward_velocity_, 
														   this->max_backward_velocity_);
	ros::param::param<float>("~maximum_turning_velocity",  this->max_turning_velocity_, 
														   this->max_turning_velocity_);

	this->DumpProfile();
	this->rate_ = (unsigned int)rate;


	// Subscribers
	this->sub_ = this->nh_.subscribe(this->topic_, 1, &RNetController::on_received_data, this); 


	return true;
}

void RNetController::set_velocity_profile(int profile) {

	int offset;
	profile--;
	this->profile_ = (RNetProfile)profile;

	offset = profile*3;

	this->max_forward_velocity_  = this->velocities_.at(offset);
	this->max_backward_velocity_ = this->velocities_.at(offset+1);
	this->max_turning_velocity_  = this->velocities_.at(offset+2);
}

void RNetController::DumpProfile(void) {

	int profile = (int)(this->profile_) + 1;
	float maxforward  = this->max_forward_velocity_;
	float maxbackward = this->max_backward_velocity_;
	float maxturning  = this->max_turning_velocity_;
	ROS_INFO("Profile %d: [%f, %f, %f]", profile, maxforward, maxbackward, maxturning);
}


void RNetController::teardown(void) {
	
	
	if (this->reader_->IsRunning()) {
		this->reader_->Stop();
		this->reader_->Join();
	}

	if (this->writer_->IsRunning()) {
		this->writer_->Stop();
		this->writer_->Join();
	}
	
	
	if (this->srv_rx_->IsRunning()) {
		this->srv_rx_->Stop();
		this->srv_rx_->Join();
	}
	
	if (this->srv_xy_->IsRunning()) {
		this->srv_xy_->Stop();
		this->srv_xy_->Join();
	}
	

}

void RNetController::Run(void) {

	bool quit = false;
	ros::Rate r(this->rate_);
	
	// Serial writer and reader services
	this->reader_->Start();
	this->writer_->Start();

	// Generic receiver service
	this->srv_rx_->Start();

	// Pause to allow receiving initial messages from the chipset
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));

	// XY service
	this->srv_xy_->Start();
	
	while(this->nh_.ok() && quit == false) {

		// Set velocity
		this->srv_xy_->SetVelocity(this->vx_, this->vy_);
		
		ros::spinOnce();
		r.sleep();
	}

}

void RNetController::on_received_data(const geometry_msgs::Twist& msg) {

	float vx, vy;

	if(std::fabs(msg.angular.z) > this->max_turning_velocity_) {
		ROS_WARN("Angular velocity over range: it will be filtered (angular = %f, max_angular = %f)", 
				 msg.angular.z, this->max_turning_velocity_);
	}

	if(msg.linear.x > this->max_forward_velocity_) {
		ROS_WARN("Forward velocity over range: it will be filtered (forward = %f, max_forward = %f)", 
				 msg.linear.x, this->max_forward_velocity_);
	}
	
	if(msg.linear.x < -this->max_backward_velocity_) {
		ROS_WARN("Backward velocity over range: it will be filtered (backward = %f, max_backward = %f)", 
				 msg.linear.x, -this->max_backward_velocity_);
	}
	
	vy = this->normalize(msg.linear.x, this->max_forward_velocity_, -this->max_backward_velocity_);
	vx = -this->normalize(msg.angular.z, this->max_turning_velocity_, -this->max_turning_velocity_);

	this->vx_ = this->filter(vx);
	this->vy_ = this->filter(vy);

	
	ROS_INFO("Message received msg.linear.x=%f | vy_ = %d", msg.linear.x, this->vy_);	
	ROS_INFO("Message received msg.angular.z=%f | vx_ = %d", msg.angular.z, this->vx_);	
}

int RNetController::filter(int value) {

	int retvalue = value;

	if(retvalue > this->max_allowed_) {
		retvalue = this->max_allowed_;
	} else if(retvalue < this->min_allowed_) {
		retvalue = this->min_allowed_;
	}

	return retvalue;
}

float RNetController::normalize(float value, float max, float min) {

	float max_allowed, min_allowed;
	
	int sign = (value > 0) ? 1 : ((value < 0) ? -1 : 0);	
	
	max_allowed = (float)this->max_allowed_;
	min_allowed = (float)this->min_allowed_;


	return ( ( ( (max_allowed - min_allowed) * (value - min) ) / (max - min) ) + min_allowed);
}

}

#endif
