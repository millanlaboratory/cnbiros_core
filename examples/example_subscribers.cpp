#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "cnbiros_core/Subscribers.hpp"

void cb_float(const std_msgs::Float32::ConstPtr& msg) {
	ROS_INFO("Received message from float topic: [%f]", msg->data);
}

void cb_string(const std_msgs::String::ConstPtr& msg) {
	ROS_INFO("Received message from string topic: [%s]", msg->data.c_str());
}

int main (int argc, char** argv) {

	std::string tfloat  = "topic_float32";
	std::string tstring = "topic_string";

	ros::init(argc, argv, "example_subscribers");

	ros::NodeHandle node;
	ros::Rate r(10);

	cnbiros::core::Subscribers subs(&node);

	subs.Add<std_msgs::Float32>(tfloat, cb_float);
	subs.Add<std_msgs::String>(tstring, cb_string);

	while(node.ok()) {

		r.sleep();
		ros::spinOnce();
	}

	return 0;
}

