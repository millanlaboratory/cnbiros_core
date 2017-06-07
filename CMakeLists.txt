set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
cmake_minimum_required(VERSION 2.8.3)
project(cnbiros_core)

find_package(catkin REQUIRED COMPONENTS roscpp)
find_package(PkgConfig)
pkg_search_module(Eigen3 REQUIRED eigen3)

SET(CMAKE_BUILD_TYPE RelWithDebInfo)

include_directories(${catkin_INCLUDE_DIRS} 
		    ${Eigen3_INCLUDE_DIRS} 
		    include)

catkin_package(
  INCLUDE_DIRS 
	include
	${Eigen3_INCLUDE_DIRS} 
  LIBRARIES 
	${PROJECT_NAME}
  CATKIN_DEPENDS
  	roscpp
  DEPENDS
  	Eigen3
)

add_library(cnbiros_core
			src/SetPublishers.cpp
			src/SetSubscribers.cpp
			)
target_link_libraries(cnbiros_core ${catkin_LIBRARIES} ${Eigen3_LIBRARY})

add_executable(example_publishers examples/example_publishers.cpp)
target_link_libraries(example_publishers cnbiros_core)

add_executable(example_subscribers examples/example_subscribers.cpp)
target_link_libraries(example_subscribers cnbiros_core)

