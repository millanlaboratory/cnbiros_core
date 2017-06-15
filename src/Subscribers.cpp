#ifndef CNBIROS_CORE_SETSUBSCRIBERS_CPP
#define CNBIROS_CORE_SETSUBSCRIBERS_CPP

#include "cnbiros_core/Subscribers.hpp"

namespace cnbiros {
	namespace core {

Subscribers::Subscribers(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

Subscribers::~Subscribers(void) {}

MapSubIt Subscribers::Find(const std::string& topic) {
	return this->rossubs_.find(topic);
}

bool Subscribers::Exist(const std::string& topic) {

	bool retcod = false;

	if( this->Find(topic) != this->rossubs_.end() )
		retcod = true;

	return retcod;
}

bool Subscribers::Remove(const std::string& topic) {

	bool retcod = false;
	MapSubIt it = this->Find(topic);

	if( it != this->rossubs_.end() ) {
		this->rossubs_.erase(it);
		retcod = true;
	}

	return retcod;
}

void Subscribers::Erase(void) {
	this->rossubs_.clear();
}

bool Subscribers::Get(const std::string& topic, ros::Subscriber*& sub) {

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
