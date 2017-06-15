#ifndef CNBIROS_CORE_NODEINTERFACE_HPP
#define CNBIROS_CORE_NODEINTERFACE_HPP

#include <ros/ros.h>

#include "cnbiros_core/InterfaceState.h"
#include "cnbiros_core/InterfaceRate.h"
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
		
		bool on_state_service_(cnbiros_core::InterfaceState::Request &req,
							   cnbiros_core::InterfaceState::Response &res);

		bool on_rate_service_(cnbiros_core::InterfaceRate::Request &req,
							  cnbiros_core::InterfaceRate::Response &res);
	public:

		static const unsigned int DoStop 	= 1;
		static const unsigned int DoResume 	= 2;
		
		static const unsigned int DoSetRate			= 1;
		static const unsigned int DoGetRate 		= 2;
		static const unsigned int DoGetCycleTime 	= 3;

	private:
		ros::NodeHandle* 	rosnode_;
		ros::Rate* 			rosrate_;
		std::string 		rosname_;
		bool 				isrunning_;

		//! Services related members
		ros::ServiceServer rossrv_state_;
		ros::ServiceServer rossrv_rate_;

};

	}
}


#endif
