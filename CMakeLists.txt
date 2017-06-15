set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_core)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs message_generation)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
		    include)

add_service_files(
  FILES
  InterfaceState.srv
  InterfaceRate.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS 
	include
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
)

add_library(cnbiros_core
			src/Publishers.cpp
			src/Subscribers.cpp
			src/NodeInterface.cpp
			)
target_link_libraries(cnbiros_core ${catkin_LIBRARIES})

add_executable(example_publishers examples/example_publishers.cpp)
target_link_libraries(example_publishers cnbiros_core)

add_executable(example_subscribers examples/example_subscribers.cpp)
target_link_libraries(example_subscribers cnbiros_core)

add_executable(example_nodeinterface examples/example_nodeinterface.cpp)
target_link_libraries(example_nodeinterface cnbiros_core)

