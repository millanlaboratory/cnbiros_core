#ifndef CNBIROS_CORE_SETPUBLISHERS_CPP
#define CNBIROS_CORE_SETPUBLISHERS_CPP

#include "cnbiros_core/SetPublishers.hpp"

namespace cnbiros {
	namespace core {

SetPublishers::SetPublishers(ros::NodeHandle* node) {
	this->rosnode_ = node;
}

SetPublishers::~SetPublishers(void) {}

MapPubIt SetPublishers::Find(const std::string& topic) {
	return this->rospubs_.find(topic);
}

bool SetPublishers::Exist(const std::string& topic) {

	bool retcod = false;

	if( this->Find(topic) != this->rospubs_.end() )
		retcod = true;

	return retcod;
}

bool SetPublishers::Remove(const std::string& topic) {

	bool retcod = false;
	MapPubIt it = this->Find(topic);

	if( it != this->rospubs_.end() ) {
		this->rospubs_.erase(it);
		retcod = true;
	}

	return retcod;
}

void SetPublishers::Erase(void) {
	this->rospubs_.clear();
}

bool SetPublishers::Get(const std::string& topic, ros::Publisher*& pub) {

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
