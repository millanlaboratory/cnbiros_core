#ifndef CNBIROS_CORE_PUBLISHERS_CPP
#define CNBIROS_CORE_PUBLISHERS_CPP

#include "cnbiros_core/Publishers.hpp"

namespace cnbiros {
	namespace core {

Publishers::Publishers(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

Publishers::~Publishers(void) {}

MapPubIt Publishers::Find(const std::string& topic) {
	return this->rospubs_.find(topic);
}

bool Publishers::Exist(const std::string& topic) {

	bool retcod = false;

	if( this->Find(topic) != this->rospubs_.end() )
		retcod = true;

	return retcod;
}

bool Publishers::Remove(const std::string& topic) {

	bool retcod = false;
	MapPubIt it = this->Find(topic);

	if( it != this->rospubs_.end() ) {
		this->rospubs_.erase(it);
		retcod = true;
	}

	return retcod;
}

void Publishers::Erase(void) {
	this->rospubs_.clear();
}

bool Publishers::Get(const std::string& topic, ros::Publisher*& pub) {

	bool retcod = false;

	MapPubIt it = this->Find(topic);
	if( it != this->rospubs_.end() ) {
		pub = &(it->second);
		retcod = true;
	}

	return retcod;
}

	}
}

#endif
