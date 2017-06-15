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
			ROS_INFO("Unknown request for %s interface", this->GetName().c_str());
			break;
	}

	return res.result;
}

bool NodeInterface::on_rate_service_(cnbiros_core::InterfaceRate::Request &req,
									 cnbiros_core::InterfaceRate::Response &res) {

	ROS_INFO("%s requested to set its rate at %f Hz", this->GetName().c_str(), req.frequency);
	res.result = this->SetRate(req.frequency);
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

	if(frequency != this->GetExpectedRate()) {
		this->rosrate_ = new ros::Rate(frequency);
		ROS_INFO("Set rate of %s to %f Hz", this->GetName().c_str(), frequency);
		this->onRateChange();
		result = true;
	} else {
		ROS_INFO("%s's rate already set at %f Hz", this->GetName().c_str(), frequency);
	}

	return result;
}

float NodeInterface::GetExpectedRate(void) {
	return float(1.0f)/this->rosrate_->expectedCycleTime().toSec();
}

float NodeInterface::GetActualRate(void) {
	return float(1.0f)/this->rosrate_->cycleTime().toSec();
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
