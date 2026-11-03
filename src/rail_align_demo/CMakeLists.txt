cmake_minimum_required(VERSION 3.0.2)
project(rail_align_demo)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs geometry_msgs sensor_msgs)

catkin_python_setup()

add_message_files(FILES AlignmentError.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(CATKIN_DEPENDS message_runtime std_msgs geometry_msgs sensor_msgs)

catkin_install_python(PROGRAMS
  scripts/generate_world.py
  scripts/spawn_robot.py
  scripts/aruco_rail_estimator.py
  scripts/alignment_controller.py
  scripts/run_recorder.py
  scripts/verify_perception.py
  scripts/verify_safety.py
  scripts/summarize_results.py
  scripts/video_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(PROGRAMS scripts/run_all_checks.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY config launch urdf DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
