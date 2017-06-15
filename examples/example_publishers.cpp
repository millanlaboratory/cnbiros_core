#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

#include "cnbiros_core/Publishers.hpp"

int main (int argc, char** argv) {

	std::string tfloat  = "topic_float32";
	std::string tstring = "topic_string";

	ros::init(argc, argv, "example_publishers");

	ros::NodeHandle node;
	ros::Rate r(10);

	cnbiros::core::Publishers pubs(&node);

	pubs.Add<std_msgs::Float32>(tfloat);
	pubs.Add<std_msgs::String>(tstring);

	std_msgs::Float32 mfloat;
	mfloat.data = 0.0f;

	std_msgs::String mstring;
	mstring.data = "test";
	
	while(node.ok()) {

		mfloat.data += 0.1f;
		pubs.Publish(tfloat, mfloat);

		pubs.Publish(tstring, mstring);
		
		r.sleep();
		ros::spinOnce();
	}

	return 0;
}

