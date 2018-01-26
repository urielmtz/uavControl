# %Tag(FULLTEXT)%
cmake_minimum_required(VERSION 2.8.3)
project(uavControl)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg image_transport cv_bridge)
find_package( OpenCV REQUIRED )

## Declare ROS messages and services
#add_message_files(FILES Num.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(uavControl src/uavControl.cpp)
target_link_libraries(uavControl ${catkin_LIBRARIES} ${OpenCV_LIBS})
add_dependencies(uavControl uavControl_generate_messages_cpp)

# %EndTag(FULLTEXT)%
