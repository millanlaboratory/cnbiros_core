#include <ros/ros.h>
#include "cnbiros_core/NodeInterface.hpp"


int main (int argc, char** argv) {

	ros::init(argc, argv, "example_nodeinterface");

	ros::NodeHandle node("~");

	cnbiros::core::NodeInterface interface(&node, "interface");

	interface.Start();

	return 0;
}

