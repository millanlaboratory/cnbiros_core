#ifndef CNBIROS_CORE_NODEINTERFACE_HPP
#define CNBIROS_CORE_NODEINTERFACE_HPP

#include <ros/ros.h>

#include "Flags.hpp"
#include "cnbiros_core/SetStateSrv.h"
#include "cnbiros_core/SetRateSrv.h"
#include "cnbiros_core/GetRateSrv.h"


namespace cnbiros {
	namespace core {

class NodeInterface  {

	public:
		NodeInterface(ros::NodeHandle* node, const std::string name);
		virtual ~NodeInterface(void);

		ros::NodeHandle* GetNode(void);

		void SetName(const std::string name);
		std::string GetName(void);

		bool SetRate(const float rate);
		float GetRate(void);
		float GetCycleTime(void);

		
		void Start(void);
		void Stop(void);
		void Run(void);
		void Resume(void);

		void Straight(void);
		void Left(void);
		void Right(void);

		bool IsRunning(void);
		

	protected:
		virtual void onRunning(void) {};
		virtual void onStraight(void) {};
		virtual void onLeft(void) {};
		virtual void onRight(void) {};
		virtual void onStop(void) {};
		virtual void onStart(void) {};
		virtual void onResume(void) {};
		virtual void onRateChange(void) {};

	private:
		
		bool on_set_state_(cnbiros_core::SetStateSrv::Request &req,
						   cnbiros_core::SetStateSrv::Response &res);

		bool on_get_rate_(cnbiros_core::GetRateSrv::Request &req,
					      cnbiros_core::GetRateSrv::Response &res);
		                                       
		bool on_set_rate_(cnbiros_core::SetRateSrv::Request &req,
						  cnbiros_core::SetRateSrv::Response &res);
		bool on_set_duration_(cnbiros_core::SetRateSrv::Request &req,
						  cnbiros_core::SetRateSrv::Response &res);
	public:

		static const unsigned int DoStart 	= 1;
		static const unsigned int DoStop 	= 2;
		static const unsigned int DoResume 	= 3;
		static const unsigned int DoPause 	= 4;
		static const unsigned int DoStraight= 5;
		static const unsigned int DoLeft 	= 6;
		static const unsigned int DoRight 	= 7;
		

	private:
		ros::NodeHandle* 	rosnode_;
		ros::Rate* 			rosrate_;
		std::string 		rosname_;
		bool 				isrunning_;
		unsigned int		mode;
		float 				duration_;
		//! Services related members
		ros::ServiceServer rossrv_set_state_;
		ros::ServiceServer rossrv_set_rate_;
		ros::ServiceServer rossrv_get_rate_;
		ros::ServiceServer rossrv_set_duration_;

};

	}
}


#endif
