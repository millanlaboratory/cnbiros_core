#ifndef CNBIROS_CORE_PUBLISHERS_HPP
#define CNBIROS_CORE_PUBLISHERS_HPP

#include <ros/ros.h>
#include <unordered_map>

#include "cnbiros_core/Flags.hpp"

namespace cnbiros {
	namespace core {

typedef std::unordered_map<std::string, ros::Publisher> MapPub;
typedef std::unordered_map<std::string, ros::Publisher>::iterator MapPubIt;
typedef std::unordered_map<std::string, ros::Publisher>::const_iterator MapPubConstIt;

class Publishers {
	public:
		Publishers(ros::NodeHandle* node);
		virtual ~Publishers(void);

		MapPubIt Find(const std::string& topic);
		bool Exist(const std::string& topic);
		bool Remove(const std::string& topic);
		void Erase(void);
		
		bool Get(const std::string& topic, ros::Publisher*& pub);

		template<class M>
		bool Add(const std::string& topic);
		
		template<class M>
		bool Publish(const std::string& topic, const M& msg);

	private:
		ros::NodeHandle* rosnode_;
		MapPub rospubs_;
};

template<class M>
bool Publishers::Add(const std::string& topic) {
	bool retcod = true;
	this->rospubs_[topic] = this->rosnode_->advertise<M>(topic, CNBIROS_CORE_BUFFER_MESSAGES);
	return retcod;
}

template<class M>
bool Publishers::Publish(const std::string& topic, const M& msg) {
	ros::Publisher* pptr;
	bool retcod = false;
	
	if(this->Get(topic, pptr) == true) {
		pptr->publish(msg);
		retcod = true;
	}
	return retcod;
}
	}
}
	


#endif


