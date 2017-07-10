#ifndef CNBIROS_CORE_NODEINTERFACE_HPP
#define CNBIROS_CORE_NODEINTERFACE_HPP

#include <ros/ros.h>

#include "cnbiros_core/SetStateSrv.h"
#include "cnbiros_core/SetRateSrv.h"
#include "cnbiros_core/GetRateSrv.h"
#include "Flags.hpp"


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
		void Resume(void);

		bool IsRunning(void);
		

	protected:
		virtual void onRunning(void) {};

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
	public:

		static const unsigned int DoStop 	= 1;
		static const unsigned int DoResume 	= 2;
		

	private:
		ros::NodeHandle* 	rosnode_;
		ros::Rate* 			rosrate_;
		std::string 		rosname_;
		bool 				isrunning_;

		//! Services related members
		ros::ServiceServer rossrv_set_state_;
		ros::ServiceServer rossrv_set_rate_;
		ros::ServiceServer rossrv_get_rate_;

};

	}
}


#endif
