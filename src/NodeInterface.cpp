#ifndef CNBIROS_CORE_NODEINTERFACE_CPP
#define CNBIROS_CORE_NODEINTERFACE_CPP

#include "cnbiros_core/NodeInterface.hpp"

namespace cnbiros {
	namespace core {

NodeInterface::NodeInterface(ros::NodeHandle* node, const std::string name) {

	// Initialize interface
	this->rosnode_		= node;
	this->rosname_ 	    = name;
	this->SetRate(CNBIROS_CORE_NODE_RATE);

	// Initialize services
	this->rossrv_state_ = this->advertiseService(this->rosnode_->getNamespace()+"/state", 
												 &NodeInterface::on_state_service_, this); 
	this->rossrv_rate_  = this->advertiseService(this->rosnode_->getNamespace()+"/rate",  
												 &NodeInterface::on_rate_service_, this); 
}

NodeInterface::~NodeInterface(void) {
	delete	this->rosrate_;
	this->rosnode_->shutdown();
}

bool NodeInterface::on_state_service_(cnbiros_core::InterfaceState::Request &req,
									  cnbiros_core::InterfaceState::Response &res) {

	res.result = false;

	switch(req.state) {
		case NodeInterface::DoStop:
			ROS_INFO("%s requested to stop", this->GetName().c_str());
			if(this->IsRunning() == true) {
				this->Stop();
				res.result = true;
			}
			break;
		case NodeInterface::DoResume:
			ROS_INFO("%s requested to resume", this->GetName().c_str());
			if(this->IsRunning() == false) {
				this->Resume();
				res.result = true;
			}
			break;
		default:
			ROS_INFO("Unknown state request for %s", this->GetName().c_str());
			break;
	}

	return res.result;
}

bool NodeInterface::on_rate_service_(cnbiros_core::InterfaceRate::Request &req,
									 cnbiros_core::InterfaceRate::Response &res) {

	res.frequency = 0.0f;
	res.time      = 0.0f;
	res.result 	  = true;

	switch(req.type) {
		case NodeInterface::DoSetRate:
			ROS_INFO("%s requested to set its rate at %f Hz", this->GetName().c_str(), req.frequency);
			if(this->SetRate(req.frequency))
				res.frequency = req.frequency;
			break;
		case NodeInterface::DoGetRate:
			ROS_INFO("%s requested to give its rate", this->GetName().c_str());
			res.frequency = this->GetRate();
			break;
		case NodeInterface::DoGetCycleTime:
			ROS_INFO("%s requested to give its cycle time", this->GetName().c_str());
			res.time = this->GetCycleTime();
			break;
		default:
			ROS_INFO("Unknown rate request for %s", this->GetName().c_str());
			res.result = false;
			break;
	}
			
	return res.result;
}

void NodeInterface::SetName(const std::string name) {
	this->rosname_ = name;
}

std::string NodeInterface::GetName(void) {
	return this->rosname_;
}

bool NodeInterface::SetRate(const float frequency) {

	bool result = false;

	if(frequency == 0.0f) {
		ROS_INFO("%s cannot set its rate at 0.0 Hz", this->GetName().c_str());
		return result;
	}

	if(frequency != this->GetRate()) {
		this->rosrate_ = new ros::Rate(frequency);
		ROS_INFO("Set rate of %s to %f Hz", this->GetName().c_str(), frequency);
		this->onRateChange();
		result = true;
	} else {
		ROS_INFO("%s's rate already set at %f Hz", this->GetName().c_str(), frequency);
	}

	return result;
}

float NodeInterface::GetRate(void) {
	return float(1.0f)/this->rosrate_->expectedCycleTime().toSec();
}

float NodeInterface::GetCycleTime(void) {
	return this->rosrate_->cycleTime().toSec();
}


bool NodeInterface::IsRunning(void) {
	return isrunning_;
}

void NodeInterface::Stop(void) {
	ROS_INFO("%s stops", this->GetName().c_str());
	this->isrunning_ = false;
	this->onStop();
}

void NodeInterface::Resume(void) {
	ROS_INFO("%s is resumed", this->GetName().c_str());
	this->isrunning_ = true;

	if(this->IsRunning())
		this->onStart();
}

void NodeInterface::Start(void) {

	ROS_INFO("%s starts", this->GetName().c_str());
	this->isrunning_ = true;
	
	while(this->ok()) {

		if(this->IsRunning())
			this->onRunning();

		rosrate_->sleep();
		ros::spinOnce();
	}
}

	}
}


#endif
