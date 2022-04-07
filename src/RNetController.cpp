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
	ros::param::param<int>("~rate", rate, 100);
	ros::param::param<float>("~maximum_forward_velocity", this->maximum_forward_velocity_, 0.27f);
	ros::param::param<float>("~maximum_backward_velocity", this->maximum_backward_velocity_, 0.27f);
	ros::param::param<float>("~maximum_turning_velocity", this->maximum_turning_velocity_, 0.27f);

	this->rate_ = (unsigned int)rate;


	// Subscribers
	this->sub_ = this->nh_.subscribe(this->topic_, 1, &RNetController::on_received_data, this); 


	return true;
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


	this->vy_ = this->normalize(msg.linear.x, this->maximum_forward_velocity_, -this->maximum_backward_velocity_);
	this->vx_ = -this->normalize(msg.angular.z, this->maximum_turning_velocity_, -this->maximum_turning_velocity_);

	this->vx_ = this->filter(this->vx_);
	this->vy_ = this->filter(this->vy_);
	
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
