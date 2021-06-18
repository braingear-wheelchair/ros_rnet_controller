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

}

RNetController::~RNetController(void) {
	
	this->teardown();

	delete this->rx_;
	delete this->tx_;
	delete this->reader_;
	delete this->writer_;
	delete this->srv_rx_;
	delete this->srv_xy_;
}

bool RNetController::Configure(void) {

	int rate;
	ros::param::param<int>("~rate", rate, 100);
	this->rate_ = (unsigned int)rate;


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

	this->teardown();

}

void RNetController::on_received_data(const geometry_msgs::Twist& msg) {


	// !!! Missing normalization !!! //
	
	this->vx_ = msg.linear.x;
	this->vy_ = msg.angular.z;
}

}

#endif
