#ifndef ROS_RNETCONTROLLER_HPP
#define ROS_RNETCONTROLLER_HPP

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <rnetserial/RNetSerial.hpp>
#include <rnetserial/RNetBuffer.hpp>
#include <rnetserial/RNetReader.hpp>
#include <rnetserial/RNetWriter.hpp>
#include <rnetserial/RNetServiceRx.hpp>
#include <rnetserial/RNetServiceXY.hpp>

namespace rosrnet {

class RNetController {

	public:
		RNetController(rnet::RNetSerial* serial);
		~RNetController(void);
		bool Configure(void);

		void Run(void);

	
	protected:
		void on_received_data(const geometry_msgs::Twist& msg);

	private:
		void teardown(void); 
	
	private:
		ros::NodeHandle			nh_;
		ros::NodeHandle			p_nh_;
		ros::Subscriber			sub_;
		unsigned int			rate_;
			
		int8_t vx_;
		int8_t vy_;
		
		rnet::RNetSerial*		serial_;
		rnet::RNetBuffer*		rx_;
		rnet::RNetBuffer*		tx_;
		rnet::RNetReader*		reader_;
		rnet::RNetWriter*		writer_;
		rnet::RNetServiceRx*	srv_rx_;
		rnet::RNetServiceXY*	srv_xy_;


};

}


#endif
