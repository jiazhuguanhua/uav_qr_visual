cmake_minimum_required(VERSION 3.0.2)
project(uav_qr_mission)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation rospy tf2_ros tf2_geometry_msgs uav_odin_qr)
catkin_python_setup()
add_message_files(FILES MissionResult.msg)
add_service_files(FILES SetTask.srv)
generate_messages(DEPENDENCIES geometry_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime rospy tf2_ros tf2_geometry_msgs uav_odin_qr)

catkin_install_python(PROGRAMS scripts/uav_qr_mission_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
