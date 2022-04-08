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

enum class RNetProfile {One, Two, Three, Four, Five};

class RNetController {

	public:
		RNetController(rnet::RNetSerial* serial);
		~RNetController(void);
		bool Configure(void);

		void Run(void);

		void DumpProfile(void);

	
	protected:
		void on_received_data(const geometry_msgs::Twist& msg);

	private:
		void teardown(void); 
		float normalize(float value, float max, float min);
		int filter(int value);
		void set_velocity_profile(int profile);
	
	private:
		ros::NodeHandle			nh_;
		ros::NodeHandle			p_nh_;
		ros::Subscriber			sub_;
		unsigned int			rate_;
		std::string 			topic_;
		
		float max_forward_velocity_;
		float max_backward_velocity_;
		float max_turning_velocity_;

		static constexpr int max_allowed_ = 100;
		static constexpr int min_allowed_ = -100;

		const std::vector<float> velocities_ = { 0.27f, 0.27f, 0.27f, 
                                           		 2.00f, 2.01f, 2.02f, 
                                           		 3.00f, 3.01f, 3.02f, 
                                           		 4.00f, 4.01f, 4.02f,
												 5.00f, 5.01f, 5.02f };

		RNetProfile profile_ = RNetProfile::One;

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
