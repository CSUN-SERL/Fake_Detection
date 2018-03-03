cmake_minimum_required(VERSION 2.8.3)
project(new_detection_msgs)

find_package(catkin REQUIRED COMPONENTS
	sensor_msgs
	std_msgs
	message_generation
	genmsg
)

add_message_files(
FILES
	CompiledMessage.msg
	Human.msg
)

generate_messages(
DEPENDENCIES
	std_msgs
	sensor_msgs
)

catkin_package(
)
