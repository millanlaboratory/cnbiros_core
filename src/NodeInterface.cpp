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
	if(ros::service::exists(ros::this_node::getName() + "/set_state", false) == false) {
		this->rossrv_set_state_ = this->rosnode_->advertiseService(ros::this_node::getName() + "/set_state",
										&NodeInterface::on_set_state_, this);
	} 

	if(ros::service::exists(ros::this_node::getName() + "/set_rate", false) == false) {
		this->rossrv_get_rate_  = this->rosnode_->advertiseService(ros::this_node::getName() + "/set_rate",
										&NodeInterface::on_set_rate_, this);
	}
	
	if(ros::service::exists(ros::this_node::getName() + "/get_rate", false) == false) {
		this->rossrv_set_rate_  = this->rosnode_->advertiseService(ros::this_node::getName() + "/get_rate",
										&NodeInterface::on_get_rate_, this);
	}

	if(ros::service::exists(ros::this_node::getName() + "/set_duration", false) == false) {
		this->rossrv_set_duration_  = this->rosnode_->advertiseService(ros::this_node::getName() + "/set_duration",
										&NodeInterface::on_set_duration_, this);
	}

	this->isrunning_ = true;
	this->mode = 0;
	this->duration_ = 5.0f;
}

NodeInterface::~NodeInterface(void) {
	delete	this->rosrate_;
	this->rosnode_->shutdown();
}

ros::NodeHandle* NodeInterface::GetNode(void) {
	return this->rosnode_;
}



bool NodeInterface::on_set_duration_(cnbiros_core::SetRateSrv::Request &req,
								  cnbiros_core::SetRateSrv::Response &res) {
	ROS_INFO("Set duration at %f s", req.frequency);
	res.result = false;
	this->duration_ = req.frequency;
	res.result - true;

	return res.result;
}


bool NodeInterface::on_set_state_(cnbiros_core::SetStateSrv::Request &req,
								  cnbiros_core::SetStateSrv::Response &res) {

	res.result = false;

	switch(req.state) {
		case NodeInterface::DoStop:
			ROS_INFO("%s requested to stop", this->GetName().c_str());
			if(this->IsRunning() == true) {
				this->Stop();
				res.result = true;
			}
			break;
		case NodeInterface::DoPause:
			ROS_INFO("%s requested to pause", this->GetName().c_str());
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
		case NodeInterface::DoStart:
			ROS_INFO("%s requested to start", this->GetName().c_str());
			if(this->IsRunning() == false) {
				this->Start();
				res.result = true;
			}
			break;
		case NodeInterface::DoStraight:
			ROS_INFO("%s requested to go straight", this->GetName().c_str());
			this->isrunning_ = true;
			this->mode = 1;
			res.result = true;
			break;
		case NodeInterface::DoLeft:
			ROS_INFO("%s requested to turn left", this->GetName().c_str());
			this->isrunning_ = true;
			this->mode = 2;
			res.result = true;
			break;
		case NodeInterface::DoRight:
			ROS_INFO("%s requested to turn right", this->GetName().c_str());
			this->isrunning_ = true;
			this->mode = 3;
			res.result = true;
			break;
		default:
			ROS_INFO("Unknown state request for %s", this->GetName().c_str());
			break;
	}

	return res.result;
}

bool NodeInterface::on_set_rate_(cnbiros_core::SetRateSrv::Request &req,
							     cnbiros_core::SetRateSrv::Response &res) {

	ROS_INFO("%s requested to set its rate at %f Hz", this->GetName().c_str(), req.frequency);
	res.result = this->SetRate(req.frequency);
			
	return res.result;
}

bool NodeInterface::on_get_rate_(cnbiros_core::GetRateSrv::Request &req,
								 cnbiros_core::GetRateSrv::Response &res) {
	res.result 	  = true;

	ROS_INFO("%s requested to give its rate info", this->GetName().c_str());
	res.frequency = this->GetRate();
	res.time = this->GetCycleTime();
			
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

	this->rosrate_ = new ros::Rate(frequency);
	ROS_INFO("Set rate of %s to %f Hz", this->GetName().c_str(), frequency);
	this->onRateChange();
	result = true;

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

void NodeInterface::Start(void) {
	ROS_INFO("%s starts", this->GetName().c_str());
	this->isrunning_ = true;

	if(this->IsRunning())
		this->onStart();
}

void NodeInterface::Resume(void) {
	ROS_INFO("%s is resumed", this->GetName().c_str());
	this->isrunning_ = true;

	//if(this->IsRunning())
	//	this->onStart();
}

void NodeInterface::Straight(void) {
	
	ros::Time 	start = ros::Time::now();

	while(this->rosnode_->ok() and (ros::Time::now()-start).toSec() < 1.0f) {
		ROS_INFO_ONCE("Prepare to go straight for %fs", this->duration_);
		rosrate_->sleep();
		ros::spinOnce();
	}
	
	while(this->rosnode_->ok() and (ros::Time::now()-start).toSec() < this->duration_) {
		ROS_INFO_ONCE("starts");

		this->onStraight();

		rosrate_->sleep();
		ros::spinOnce();
	}
	this->isrunning_ = false;

}

void NodeInterface::Left(void) {
	
	ros::Time 	start = ros::Time::now();

	while(this->rosnode_->ok() and (ros::Time::now()-start).toSec() < 1.0f) {
		ROS_INFO_ONCE("Prepare to turn left for %fs", this->duration_);
		rosrate_->sleep();
		ros::spinOnce();
	}
	
	while(this->rosnode_->ok() and (ros::Time::now()-start).toSec() < this->duration_) {
		ROS_INFO_ONCE("starts");

		this->onLeft();

		rosrate_->sleep();
		ros::spinOnce();
	}
	ROS_INFO("stops");
	this->isrunning_ = false;

}

void NodeInterface::Right(void) {
	
	ros::Time 	start = ros::Time::now();

	while(this->rosnode_->ok() and (ros::Time::now()-start).toSec() < 1.0f) {
		ROS_INFO_ONCE("Prepare to turn right for %fs", this->duration_);
		rosrate_->sleep();
		ros::spinOnce();
	}
	
	while(this->rosnode_->ok() and (ros::Time::now()-start).toSec() < this->duration_) {
		ROS_INFO_ONCE("starts");

		this->onRight();

		rosrate_->sleep();
		ros::spinOnce();
	}

	this->isrunning_ = false;

}


void NodeInterface::Run(void) {

	while(this->rosnode_->ok()) {

		ROS_INFO_ONCE("%s starts", this->GetName().c_str());
		if(this->IsRunning())

			switch(this->mode) {
				case 0:
					this->onRunning();
					break;
				case 1:
					Straight();
					break;
				case 2:
					Left();
					break;
				case 3:
					Right();
					break;
			}
			

		rosrate_->sleep();
		ros::spinOnce();
	}
}

	}
}


#endif
