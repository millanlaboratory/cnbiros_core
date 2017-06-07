#ifndef CNBIROS_CORE_SETSUBSCRIBERS_CPP
#define CNBIROS_CORE_SETSUBSCRIBERS_CPP

#include "cnbiros_core/SetSubscribers.hpp"

namespace cnbiros {
	namespace core {

SetSubscribers::SetSubscribers(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

SetSubscribers::~SetSubscribers(void) {}

MapSubIt SetSubscribers::Find(const std::string& topic) {
	return this->rossubs_.find(topic);
}

bool SetSubscribers::Exist(const std::string& topic) {

	bool retcod = false;

	if( this->Find(topic) != this->rossubs_.end() )
		retcod = true;

	return retcod;
}

bool SetSubscribers::Remove(const std::string& topic) {

	bool retcod = false;
	MapSubIt it = this->Find(topic);

	if( it != this->rossubs_.end() ) {
		this->rossubs_.erase(it);
		retcod = true;
	}

	return retcod;
}

void SetSubscribers::Erase(void) {
	this->rossubs_.clear();
}

bool SetSubscribers::Get(const std::string& topic, ros::Subscriber*& sub) {

	bool retcod = false;

	MapSubIt it = this->Find(topic);
	if( it != this->rossubs_.end() ) {
		sub = &(it->second);
		retcod = true;
	}

	return retcod;
}

	}
}

#endif
