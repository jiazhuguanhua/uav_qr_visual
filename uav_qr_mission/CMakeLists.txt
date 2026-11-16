cmake_minimum_required(VERSION 3.0.2)
project(uav_qr_mission)

find_package(catkin REQUIRED COMPONENTS geometry_msgs sensor_msgs std_msgs message_generation rospy tf2_ros tf2_geometry_msgs uav_odin_qr)
catkin_python_setup()
add_message_files(FILES MissionResult.msg QrTargetResult.msg QrReport.msg QrReportStatus.msg)
add_service_files(FILES SetTask.srv CaptureQr.srv FinishSurvey.srv ReportQr.srv)
generate_messages(DEPENDENCIES geometry_msgs sensor_msgs std_msgs)
catkin_package(CATKIN_DEPENDS geometry_msgs sensor_msgs std_msgs message_runtime rospy tf2_ros tf2_geometry_msgs uav_odin_qr)

catkin_install_python(PROGRAMS scripts/uav_qr_mission_node scripts/qr_report_sender
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config docs DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
