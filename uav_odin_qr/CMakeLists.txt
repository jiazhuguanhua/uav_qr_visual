cmake_minimum_required(VERSION 3.0.2)
project(uav_odin_qr)

find_package(catkin REQUIRED COMPONENTS geometry_msgs message_generation std_msgs uav_oyo)
catkin_python_setup()

add_message_files(FILES Detection.msg DetectionArray.msg QrTransform.msg QrTransformArray.msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs message_runtime std_msgs uav_oyo)

catkin_install_python(PROGRAMS scripts/uav_odin_qr_node DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
